#!/usr/bin/env python3
"""
main.py -- read-only monitor for the Vescent RUBRIComb, SLICE-FPGA, and one
or more SLICE-OPLs.

Instruments, serial/network settings, and InfluxDB connection are all defined
in a YAML config file rather than passed on the command line, so a lab
running several SLICE-OPL channels doesn't need a wall of flags -- each unit
just gets an entry with its own name (Influx measurement) and COM port. See
config.yaml for the full schema.

STRICTLY READ-ONLY
------------------
This script cannot change instrument state:
  * every device is constructed with read_only=True, which makes the transport
    layer refuse any command that is not on the read allowlist -- setters,
    *RST, #SAVESETTINGS, NOCP, DDSAUTO, _SELFCAL and _FACTORY all raise
    ReadOnlyError before a byte is transmitted;
  * no startup, shutdown, or laser-enable sequence is invoked;
  * serial ports are opened with DTR and RTS deasserted, so opening the port
    does not pulse a reset into the USB front end.
Running it twice, or interrupting it mid-sweep, leaves the hardware exactly as
it was.

One caveat for the SLICE-FPGA: its command set is undocumented, so its
read_only guard is a heuristic (blocks anything not ending in '?') rather than
the serial drivers' allowlist. That's still enough to cover this script, since
it only ever sends the '?'-terminated commands in slice_fpga.MONITOR_PARAMS.

Usage
-----
    # one snapshot of every configured instrument, printed, nothing written
    python main.py --once --dry-run

    # continuous logging to Influx every 10 s (interval set in the config)
    python main.py -v

    # use a config file that isn't ./config.yaml
    python main.py --config /path/to/other_config.yaml
"""

from __future__ import annotations

import argparse
import logging
import sys
import threading
from typing import Any, Dict, List, Optional, Sequence

import yaml

from rubricomb import RubriComb
from slice_fpga import SliceFPGA
from slice_opl import SliceOPL
from vescent_serial import (
    BatchedInfluxSink,
    ConsoleSink,
    InfluxConfig,
    MonitoredDevice,
    PerFieldInfluxSink,
    VescentError,
    monitor,
    sweep_once,
)

log = logging.getLogger("vescent.main")

DEFAULT_SERIAL: Dict[str, Any] = dict(baud=115200, timeout=2.0, assert_dtr=False)


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Read-only monitor for the Vescent RUBRIComb and one or "
                    "more SLICE-OPLs, configured from a YAML file",
    )
    p.add_argument("-c", "--config", default="config.yaml",
                   help="path to the YAML config file (default: config.yaml)")
    p.add_argument("--once", action="store_true", help="take one sweep and exit")
    p.add_argument("--dry-run", action="store_true",
                   help="print sweeps instead of writing to Influx")
    p.add_argument("-v", "--verbose", action="count", default=0)
    return p


def load_config(path: str) -> Dict[str, Any]:
    """Load and parse the YAML config file. Raises on missing/invalid file."""
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def _serial_kwargs(cfg: Dict[str, Any], entry: Dict[str, Any]) -> Dict[str, Any]:
    """Per-device serial settings: entry overrides fall back to the
    top-level 'serial' defaults, which fall back to DEFAULT_SERIAL."""
    common = {**DEFAULT_SERIAL, **cfg.get("serial", {})}
    return dict(
        baudrate=entry.get("baud", common["baud"]),
        timeout=entry.get("timeout", common["timeout"]),
        assert_dtr=entry.get("assert_dtr", common["assert_dtr"]),
        read_only=True,   # not configurable on purpose: this script only reads
    )


def make_sink(cfg: Dict[str, Any], dry_run: bool) -> Any:
    if dry_run:
        return ConsoleSink()
    influx_cfg = cfg.get("influx") or {}
    # url/org come from the INFLUX_URL / INFLUX_ORG env vars (InfluxConfig
    # falls back to its own defaults if they're unset) -- not from the config
    # file, so they don't have to be duplicated per machine.
    icfg = InfluxConfig(
        bucket=influx_cfg.get("bucket", "vescent-demo"),
        token=influx_cfg.get("token"),
    )
    if influx_cfg.get("mode", "per-field") == "batched":
        device_tag = influx_cfg.get("device_tag", "")
        tags = {"host": device_tag} if device_tag else None
        return BatchedInfluxSink(icfg, tags=tags)
    return PerFieldInfluxSink(icfg)


def build_devices(cfg: Dict[str, Any]) -> Optional[List[MonitoredDevice]]:
    """Construct and open every configured device.

    Returns None (after printing a message to stderr) if the config is
    invalid; callers should return exit code 2 in that case.
    """
    rubricomb_cfg = cfg.get("rubricomb")
    slice_fpga_cfg = cfg.get("slice_fpga")
    slice_opl_cfgs = cfg.get("slice_opls") or []

    if not rubricomb_cfg and not slice_fpga_cfg and not slice_opl_cfgs:
        print("No instruments configured -- add a 'rubricomb', 'slice_fpga', "
              "and/or 'slice_opls' section to the config file.", file=sys.stderr)
        return None

    names = [entry.get("name") for entry in slice_opl_cfgs]
    if len(names) != len(set(names)):
        print("Duplicate 'name' among slice_opls entries -- each SLICE-OPL "
              "needs a unique name (it's used as the Influx measurement).",
              file=sys.stderr)
        return None

    devices: List[MonitoredDevice] = []
    if rubricomb_cfg:
        if not rubricomb_cfg.get("port"):
            print("rubricomb section is missing 'port'", file=sys.stderr)
            return None
        comb = RubriComb(rubricomb_cfg["port"], **_serial_kwargs(cfg, rubricomb_cfg))
        devices.append(MonitoredDevice(rubricomb_cfg.get("measurement", "rubricomb"), comb))

    if slice_fpga_cfg:
        fpga = SliceFPGA(
            host=slice_fpga_cfg.get("host", SliceFPGA.DEFAULT_HOST),
            port=slice_fpga_cfg.get("port", SliceFPGA.DEFAULT_PORT),
            timeout=slice_fpga_cfg.get("timeout", 2.0),
            read_only=True,   # not configurable on purpose: this script only reads
        )
        devices.append(MonitoredDevice(slice_fpga_cfg.get("measurement", "slice_fpga"), fpga))

    for entry in slice_opl_cfgs:
        name, port = entry.get("name"), entry.get("port")
        if not name or not port:
            print(f"slice_opls entry missing 'name' or 'port': {entry}",
                  file=sys.stderr)
            for d in devices:
                d.device.close()
            return None
        opl = SliceOPL(port, **_serial_kwargs(cfg, entry))
        devices.append(MonitoredDevice(name, opl))

    return devices


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose >= 2 else
              logging.INFO if args.verbose == 1 else logging.WARNING,
        format="%(asctime)s %(levelname)-7s %(message)s",
    )

    try:
        cfg = load_config(args.config)
    except FileNotFoundError:
        print(f"Config file not found: {args.config}\n"
              f"See config.yaml for the expected format, or pass --config "
              f"to point elsewhere.",
              file=sys.stderr)
        return 2
    except yaml.YAMLError as exc:
        print(f"Could not parse {args.config}: {exc}", file=sys.stderr)
        return 2

    devices = build_devices(cfg)
    if devices is None:
        return 2

    try:
        # Identify each box up front so a swapped cable is obvious immediately.
        for entry in devices:
            try:
                print(f"{entry.measurement}: {entry.device.idn()}")
            except VescentError as exc:
                where = getattr(entry.device, "address", entry.device.port)
                log.error("[%s] identification failed on %s: %s",
                          entry.measurement, where, exc)

        sink = make_sink(cfg, args.dry_run)
        stop = threading.Event()
        try:
            if args.once:
                for entry in devices:
                    n_ok, n_fail = sweep_once(entry, sink)
                    print(f"{entry.measurement}: {n_ok} fields, {n_fail} failures",
                          file=sys.stderr)
            else:
                interval = (cfg.get("polling") or {}).get("interval", 10.0)
                monitor(devices, sink, interval=interval, stop_event=stop)
        except KeyboardInterrupt:
            print("\nStopping.", file=sys.stderr)
            stop.set()
        finally:
            sink.close()
    finally:
        for entry in devices:
            entry.device.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
