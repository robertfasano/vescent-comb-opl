#!/usr/bin/env python3
"""
main.py -- read-only monitor for the Vescent RUBRIComb and SLICE-OPL.

Connects to both instruments, sweeps every parameter in each driver's
MONITOR_PARAMS table, and writes the results to InfluxDB.

STRICTLY READ-ONLY
------------------
This script cannot change instrument state:
  * both devices are constructed with read_only=True, which makes the transport
    layer refuse any command that is not on the read allowlist -- setters,
    *RST, #SAVESETTINGS, NOCP, DDSAUTO, _SELFCAL and _FACTORY all raise
    ReadOnlyError before a byte is transmitted;
  * no startup, shutdown, or laser-enable sequence is invoked;
  * serial ports are opened with DTR and RTS deasserted, so opening the port
    does not pulse a reset into the USB front end.
Running it twice, or interrupting it mid-sweep, leaves the hardware exactly as
it was.

Usage
-----
    # one snapshot of both boxes, printed, nothing written
    python main.py --rubricomb-port /dev/ttyUSB0 --opl-port /dev/ttyUSB1 \
        --once --dry-run

    # continuous logging to Influx every 10 s
    python main.py --rubricomb-port /dev/ttyUSB0 --opl-port /dev/ttyUSB1 \
        --interval 10 -v

    # just one of the two
    python main.py --opl-port /dev/ttyUSB1 --once
"""

from __future__ import annotations

import argparse
import logging
import sys
import threading
from typing import Any, List, Optional, Sequence

from rubricomb import RubriComb
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


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Read-only monitor for the Vescent RUBRIComb and SLICE-OPL",
    )
    # -- instruments ---------------------------------------------------------
    p.add_argument("--rubricomb-port", help="e.g. /dev/ttyUSB0 or COM4")
    p.add_argument("--opl-port", help="e.g. /dev/ttyUSB1 or COM5")
    p.add_argument("--rubricomb-measurement", default="rubricomb")
    p.add_argument("--opl-measurement", default="slice_opl")
    p.add_argument("--baud", type=int, default=115200, help="9600-115200 (default: 115200)")
    p.add_argument("--timeout", type=float, default=2.0, help="serial read timeout [s]")
    p.add_argument("--assert-dtr", action="store_true",
                   help="assert DTR/RTS on open (default: leave deasserted)")

    # -- polling -------------------------------------------------------------
    p.add_argument("--interval", type=float, default=10.0, help="polling interval [s]")
    p.add_argument("--once", action="store_true", help="take one sweep and exit")

    # -- output --------------------------------------------------------------
    p.add_argument("--dry-run", action="store_true",
                   help="print sweeps instead of writing to Influx")
    p.add_argument("--influx-mode", choices=("per-field", "batched"),
                   default="per-field",
                   help="per-field uses the attached write_point helper (default); "
                        "batched writes one point per device per sweep on a "
                        "persistent client")
    p.add_argument("--bucket", default="Yb2")
    p.add_argument("--org", default="yblab")
    p.add_argument("--url", default="http://localhost:8086")
    p.add_argument("--device-tag", default="", help="extra Influx tag (batched mode)")

    p.add_argument("-v", "--verbose", action="count", default=0)
    return p


def make_sink(args: argparse.Namespace) -> Any:
    if args.dry_run:
        return ConsoleSink()
    cfg = InfluxConfig(url=args.url, org=args.org, bucket=args.bucket)
    if args.influx_mode == "batched":
        tags = {"host": args.device_tag} if args.device_tag else None
        return BatchedInfluxSink(cfg, tags=tags)
    return PerFieldInfluxSink(cfg)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose >= 2 else
              logging.INFO if args.verbose == 1 else logging.WARNING,
        format="%(asctime)s %(levelname)-7s %(message)s",
    )

    if not args.rubricomb_port and not args.opl_port:
        print("Specify at least one of --rubricomb-port / --opl-port",
              file=sys.stderr)
        return 2

    common = dict(
        baudrate=args.baud,
        timeout=args.timeout,
        assert_dtr=args.assert_dtr,
        read_only=True,   # not configurable on purpose: this script only reads
    )

    devices: List[MonitoredDevice] = []
    opened: List[Any] = []
    try:
        if args.rubricomb_port:
            comb = RubriComb(args.rubricomb_port, **common)
            opened.append(comb)
            devices.append(MonitoredDevice(args.rubricomb_measurement, comb))
        if args.opl_port:
            opl = SliceOPL(args.opl_port, **common)
            opened.append(opl)
            devices.append(MonitoredDevice(args.opl_measurement, opl))

        # Identify each box up front so a swapped cable is obvious immediately.
        for entry in devices:
            try:
                print(f"{entry.measurement}: {entry.device.idn()}")
            except VescentError as exc:
                log.error("[%s] identification failed on %s: %s",
                          entry.measurement, entry.device.port, exc)

        sink = make_sink(args)
        stop = threading.Event()
        try:
            if args.once:
                for entry in devices:
                    n_ok, n_fail = sweep_once(entry, sink)
                    print(f"{entry.measurement}: {n_ok} fields, {n_fail} failures",
                          file=sys.stderr)
            else:
                monitor(devices, sink, interval=args.interval, stop_event=stop)
        except KeyboardInterrupt:
            print("\nStopping.", file=sys.stderr)
            stop.set()
        finally:
            sink.close()
    finally:
        for device in opened:
            device.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
