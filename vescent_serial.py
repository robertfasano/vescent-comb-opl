#!/usr/bin/env python3
"""
vescent_serial.py -- shared serial transport, parsing, and InfluxDB plumbing
for Vescent ASCII-API instruments (RUBRIComb, SLICE-OPL).

Both instruments speak the same protocol: 8N1, no flow control, CR-terminated
case-insensitive ASCII commands of the form

    [command name] [parameter] [parameter] [parameter]

so the transport, echo handling, response parsing, parameter-table sweep, and
Influx sinks live here. Instrument-specific command sets live in rubricomb.py
and slice_opl.py.

READ-ONLY BY DEFAULT
--------------------
VescentSerialDevice refuses to transmit any command that is not a pure query
unless it is constructed with read_only=False. The check is an allowlist, not a
guess at punctuation: SLICE-OPL's READVOLT is a read with no '?', while NOCP,
DDSAUTO and _FACTORY change state with no '?' either.
"""

from __future__ import annotations

import datetime
import logging
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

try:  # pyserial is only needed to talk to real hardware
    import serial  # type: ignore
except ImportError:  # pragma: no cover
    serial = None  # type: ignore

log = logging.getLogger("vescent")


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------

class VescentError(Exception):
    """Base exception for Vescent serial drivers."""


class NoResponseError(VescentError):
    """The instrument did not reply before the read timeout expired."""


class ResponseParseError(VescentError):
    """The instrument replied with something we could not interpret."""


class ReadOnlyError(VescentError):
    """A state-changing command was attempted on a read-only connection."""


# ---------------------------------------------------------------------------
# Response parsers
# ---------------------------------------------------------------------------

def _first_token(text: str) -> str:
    parts = text.strip().split()
    if not parts:
        raise ResponseParseError("empty response")
    return parts[0]


def parse_float(text: str) -> float:
    try:
        return float(_first_token(text))
    except ValueError as exc:
        raise ResponseParseError(f"expected a float, got {text!r}") from exc


def parse_int(text: str) -> int:
    tok = _first_token(text)
    try:
        return int(tok, 0)
    except ValueError:
        try:  # some firmware paths return "49152.000000"
            return int(float(tok))
        except ValueError as exc:
            raise ResponseParseError(f"expected an int, got {text!r}") from exc


def parse_bool(text: str) -> bool:
    """Parse On/Off (and 1/0) replies into a bool."""
    tok = _first_token(text).strip().lower().rstrip(",")
    if tok in ("on", "1", "true", "enabled", "yes"):
        return True
    if tok in ("off", "0", "false", "disabled", "no"):
        return False
    raise ResponseParseError(f"expected On/Off, got {text!r}")


def parse_str(text: str) -> str:
    return text.strip()


# ---------------------------------------------------------------------------
# Parameter table entry
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Param:
    """A single readable parameter.

    key      : InfluxDB field name
    command  : full serial query string (command + any argument)
    parser   : callable turning the reply text into a Python value
    group    : subsystem tag, for readability when scanning the table
    unit     : physical unit, documentation only
    """
    key: str
    command: str
    parser: Callable[[str], Any]
    group: str = ""
    unit: str = ""


# ---------------------------------------------------------------------------
# Base driver
# ---------------------------------------------------------------------------

class VescentSerialDevice:
    """Common serial layer for Vescent ASCII-API instruments.

    Subclasses set MONITOR_PARAMS (and optionally READ_SAFE_COMMANDS and
    derived_fields()).
    """

    TERMINATOR = b"\r"

    #: Parameter table swept by read_all(); overridden per instrument.
    MONITOR_PARAMS: Tuple[Param, ...] = ()

    #: Commands that read state but do not end in '?'. Anything not ending in
    #: '?' and not listed here is treated as state-changing.
    READ_SAFE_COMMANDS: frozenset = frozenset()

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 115200,
        timeout: float = 2.0,
        write_timeout: float = 2.0,
        inter_command_delay: float = 0.02,
        read_only: bool = True,
        assert_dtr: bool = False,
        transport: Any = None,
        open_on_init: bool = True,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.inter_command_delay = inter_command_delay
        self.read_only = read_only
        self.assert_dtr = assert_dtr
        self._ser = transport
        self._lock = threading.RLock()
        if transport is None and open_on_init and port is not None:
            self.open()

    # -- connection management ----------------------------------------------

    def open(self) -> None:
        """Open the serial port.

        DTR and RTS are left deasserted by default: opening a USB CDC port with
        DTR asserted pulses the line, which some microcontroller front ends
        interpret as a reset. Pass assert_dtr=True if your unit needs them.
        """
        if self._ser is not None and getattr(self._ser, "is_open", False):
            return
        if serial is None:
            raise VescentError("pyserial is not installed -- run `pip install pyserial`")
        log.info("Opening %s at %d baud (read_only=%s)", self.port, self.baudrate,
                 self.read_only)
        ser = serial.Serial()
        ser.port = self.port
        ser.baudrate = self.baudrate
        ser.bytesize = 8          # 8 data bits
        ser.parity = "N"          # no parity
        ser.stopbits = 1          # 1 stop bit
        ser.rtscts = False        # no flow control
        ser.xonxoff = False
        ser.dsrdtr = False
        ser.timeout = self.timeout
        ser.write_timeout = self.write_timeout
        if not self.assert_dtr:
            ser.dtr = False
            ser.rts = False
        ser.open()
        self._ser = ser
        time.sleep(0.1)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # pragma: no cover
                pass

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._ser is not None and bool(getattr(self._ser, "is_open", False))

    # -- read-only enforcement ----------------------------------------------

    @classmethod
    def is_query(cls, command: str) -> bool:
        """True if *command* only reads state.

        A command is a read if its command token ends in '?' or is listed in
        READ_SAFE_COMMANDS. Note that '?' alone is not a reliable test in
        either direction on these instruments.
        """
        tokens = command.strip().split()
        if not tokens:
            return False
        name = tokens[0].upper()
        return name.endswith("?") or name in cls.READ_SAFE_COMMANDS

    def _require_writable(self, action: str) -> None:
        """Fail fast at the top of a state-changing sequence.

        Sequences that begin by reading (startup(), for example) could
        otherwise return early on a read-only connection and look as though
        they succeeded.
        """
        if self.read_only:
            raise ReadOnlyError(
                f"refusing to run {action}: this connection is read-only. "
                f"Construct {type(self).__name__}(..., read_only=False) to "
                f"allow state changes."
            )

    def _check_read_only(self, command: str) -> None:
        if self.read_only and not self.is_query(command):
            raise ReadOnlyError(
                f"refusing to send {command.strip()!r}: this connection is "
                f"read-only. Construct {type(self).__name__}(..., read_only=False) "
                f"to allow state changes."
            )

    # -- low-level transport -------------------------------------------------

    def _read_line(self) -> str:
        """Read one CR- or LF-terminated line. Returns '' on timeout."""
        buf = bytearray()
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            chunk = self._ser.read(1)
            if not chunk:
                continue
            if chunk in (b"\r", b"\n"):
                if buf:
                    break
                continue  # swallow leading/duplicate terminators
            buf += chunk
        return buf.decode("ascii", errors="replace").strip()

    @staticmethod
    def _strip_echo(line: str, sent: str) -> Optional[str]:
        """Remove command echo. Returns None if the line was pure echo.

        Handles all three reply shapes these instruments use: bare value
        ("24.21"), echo-plus-value ("MSTRCTL? 0", "#SCBKLT? 5"), and pure echo.
        """
        line_s, sent_s = line.strip(), sent.strip()
        if line_s.upper() == sent_s.upper():
            return None
        if line_s.upper().startswith(sent_s.upper()):
            remainder = line_s[len(sent_s):].strip()
            return remainder or None
        return line_s

    def _transact(self, command: str, expect_reply: bool = True,
                  max_lines: int = 4, allow_echo_reply: bool = False) -> str:
        """Send one command and return the first non-echo reply line.

        Some commands are documented to return only an echo of what was sent;
        pass allow_echo_reply=True so that echo counts as the reply.
        """
        self._check_read_only(command)
        if not self.is_open:
            raise VescentError("serial port is not open")
        with self._lock:
            payload = command.strip().encode("ascii") + self.TERMINATOR
            log.debug("TX: %s", command.strip())
            self._ser.reset_input_buffer()
            self._ser.write(payload)
            self._ser.flush()
            if not expect_reply:
                time.sleep(self.inter_command_delay)
                return ""
            last_echo: Optional[str] = None
            for _ in range(max_lines):
                raw = self._read_line()
                if not raw:
                    break
                log.debug("RX: %s", raw)
                stripped = self._strip_echo(raw, command)
                if stripped is not None:
                    time.sleep(self.inter_command_delay)
                    return stripped
                last_echo = raw.strip()
            time.sleep(self.inter_command_delay)
            if allow_echo_reply and last_echo is not None:
                return last_echo
            raise NoResponseError(f"no reply to {command!r}")

    # -- public generic access ----------------------------------------------

    def query(self, command: str, parser: Callable[[str], Any] = parse_str) -> Any:
        """Send a read command and parse the reply."""
        return parser(self._transact(command))

    def write(self, command: str, expect_reply: bool = True) -> str:
        """Send a command; most setters echo back the resulting value.

        Blocked unless the device was constructed with read_only=False.
        """
        return self._transact(command, expect_reply=expect_reply,
                              allow_echo_reply=True)

    # -- shared commands -----------------------------------------------------

    def idn(self) -> str:
        """Manufacturer, model, serial number, firmware versions."""
        return self.query("*IDN?")

    def reset(self) -> str:
        """Reboot the device. Blocked on a read-only connection."""
        return self._transact("*RST", max_lines=2, allow_echo_reply=True)

    # -- sweep ---------------------------------------------------------------

    def derived_fields(self, values: Dict[str, Any]) -> Dict[str, Any]:
        """Hook: compute extra fields from a completed sweep. Override."""
        return {}

    def read_all(
        self,
        params: Optional[Sequence[Param]] = None,
        include_derived: bool = True,
    ) -> Tuple[Dict[str, Any], Dict[str, str]]:
        """Poll every parameter once.

        Returns ``(values, failures)`` where *failures* maps field name to the
        error text. A single bad reply never aborts the sweep -- partial data
        beats no data when you are trying to see what the instrument did at
        3 a.m.
        """
        params = self.MONITOR_PARAMS if params is None else params
        values: Dict[str, Any] = {}
        failures: Dict[str, str] = {}
        for p in params:
            try:
                values[p.key] = self.query(p.command, p.parser)
            except (VescentError, ValueError, OSError) as exc:
                failures[p.key] = f"{type(exc).__name__}: {exc}"
                log.warning("Failed to read %s (%s): %s", p.key, p.command, exc)
        if include_derived and values:
            try:
                values.update(self.derived_fields(values))
            except Exception as exc:  # never let a derived field kill a sweep
                log.warning("Derived field computation failed: %s", exc)
        return values, failures


# ---------------------------------------------------------------------------
# InfluxDB sinks
# ---------------------------------------------------------------------------

class InfluxConfig:
    """Connection settings, overridable from the environment."""

    def __init__(
        self,
        url: str = "http://localhost:8086",
        token: Optional[str] = None,
        org: str = "yblab",
        bucket: str = "vescent-demo",
    ) -> None:
        self.url = os.environ.get("INFLUX_URL", url)
        self.token = token or os.environ.get("INFLUX_TOKEN", "INSERT YOUR TOKEN HERE")
        self.org = os.environ.get("INFLUX_ORG", org)
        self.bucket = os.environ.get("INFLUX_BUCKET", bucket)


def coerce_for_influx(value: Any) -> Any:
    """Keep field types stable across writes.

    InfluxDB rejects a point whose field type differs from the type already
    recorded for that field, so every numeric value is written as a float and
    only bools stay bools.
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return float(value)
    return value


class PerFieldInfluxSink:
    """Writes one point per field using the supplied ``write_point`` helper
    (influx_example.write_point, imported when available).

    Simple and drop-in, but it opens and tears down an InfluxDBClient for every
    field -- ~30 connections per RUBRIComb sweep, ~55 per SLICE-OPL sweep. Fine
    at 10 s cadence on localhost; use BatchedInfluxSink if you tighten that.
    """

    def __init__(self, config: InfluxConfig, writer: Optional[Callable] = None) -> None:
        self.config = config
        self._write_point = writer or self._import_attached_writer()

    @staticmethod
    def _import_attached_writer() -> Callable:
        try:
            from influx_example import write_point  # type: ignore
            return write_point
        except ImportError:
            return _fallback_write_point

    def write(self, measurement: str, values: Dict[str, Any], timestamp=None) -> None:
        for field_name, value in values.items():
            self._write_point(measurement, field_name, coerce_for_influx(value))

    def close(self) -> None:  # symmetry with BatchedInfluxSink
        pass


def _fallback_write_point(measurement: str, field_name: str, value: Any) -> None:
    """Same shape as influx_example.write_point, used when that module is not
    importable."""
    from influxdb_client import InfluxDBClient, Point, WritePrecision
    from influxdb_client.client.write_api import SYNCHRONOUS

    cfg = InfluxConfig()
    try:
        client = InfluxDBClient(url=cfg.url, token=cfg.token, org=cfg.org)
    except Exception as exc:
        log.error("Error connecting to InfluxDB: %s", exc)
        return
    try:
        write_api = client.write_api(write_options=SYNCHRONOUS)
        point = (
            Point(measurement)
            .field(field_name, value)
            .time(datetime.datetime.now(datetime.timezone.utc), WritePrecision.NS)
        )
        write_api.write(bucket=cfg.bucket, org=cfg.org, record=point)
    except Exception as exc:
        log.error("Error writing data: %s", exc)
    finally:
        client.close()


class BatchedInfluxSink:
    """Writes each device's sweep as a single point on a persistent client.

    Recommended for continuous logging: one connection, one point per device
    per sweep, all fields sharing an identical timestamp so Flux joins across
    channels line up exactly.
    """

    def __init__(self, config: InfluxConfig, tags: Optional[Dict[str, str]] = None) -> None:
        from influxdb_client import InfluxDBClient
        from influxdb_client.client.write_api import SYNCHRONOUS

        self.config = config
        self.tags = tags or {}
        self._client = InfluxDBClient(url=config.url, token=config.token, org=config.org)
        self._write_api = self._client.write_api(write_options=SYNCHRONOUS)

    def write(self, measurement: str, values: Dict[str, Any], timestamp=None) -> None:
        from influxdb_client import Point, WritePrecision

        if not values:
            return
        point = Point(measurement)
        for tag_key, tag_val in self.tags.items():
            point = point.tag(tag_key, tag_val)
        for field_name, value in values.items():
            point = point.field(field_name, coerce_for_influx(value))
        stamp = timestamp or datetime.datetime.now(datetime.timezone.utc)
        point = point.time(stamp, WritePrecision.NS)
        self._write_api.write(bucket=self.config.bucket, org=self.config.org,
                              record=point)

    def close(self) -> None:
        try:
            self._client.close()
        except Exception:  # pragma: no cover
            pass


class ConsoleSink:
    """Dry-run sink: prints each sweep instead of writing to Influx."""

    def write(self, measurement: str, values: Dict[str, Any], timestamp=None) -> None:
        stamp = timestamp or datetime.datetime.now(datetime.timezone.utc)
        print(f"--- {measurement} @ {stamp.isoformat()} ---")
        for k, v in values.items():
            print(f"  {k:28s} {v}")

    def close(self) -> None:
        pass


# ---------------------------------------------------------------------------
# Monitoring loop
# ---------------------------------------------------------------------------

@dataclass
class MonitoredDevice:
    """One instrument in the polling loop."""
    measurement: str
    device: VescentSerialDevice
    params: Optional[Sequence[Param]] = None


def sweep_once(entry: MonitoredDevice, sink: Any) -> Tuple[int, int]:
    """Poll one device and hand the result to *sink*. Returns (n_ok, n_failed)."""
    stamp = datetime.datetime.now(datetime.timezone.utc)
    try:
        values, failures = entry.device.read_all(entry.params)
    except Exception as exc:
        log.error("[%s] sweep failed: %s", entry.measurement, exc)
        return 0, -1
    values["read_failures"] = float(len(failures))
    try:
        sink.write(entry.measurement, values, timestamp=stamp)
    except Exception as exc:
        log.error("[%s] Influx write failed: %s", entry.measurement, exc)
    return len(values), len(failures)


def monitor(
    devices: Sequence[MonitoredDevice],
    sink: Any,
    interval: float = 10.0,
    stop_event: Optional[threading.Event] = None,
    reconnect: bool = True,
) -> None:
    """Poll every device on a fixed cadence and push results to *sink*.

    Scheduling is drift-free: each sweep is aligned to a monotonic grid rather
    than sleeping a fixed amount after variable-duration work.
    """
    stop_event = stop_event or threading.Event()
    next_tick = time.monotonic()

    while not stop_event.is_set():
        next_tick += interval
        for entry in devices:
            n_ok, n_fail = sweep_once(entry, sink)
            if n_fail < 0 and reconnect:
                _try_reconnect(entry.device)
            else:
                log.info("[%s] logged %d fields (%d failures)",
                         entry.measurement, n_ok, n_fail)
        sleep_for = next_tick - time.monotonic()
        if sleep_for < 0:
            log.warning("Sweep took longer than the %.1f s interval", interval)
            next_tick = time.monotonic()
            sleep_for = 0
        stop_event.wait(sleep_for)


def _try_reconnect(device: VescentSerialDevice, delay: float = 5.0) -> None:
    log.info("Attempting to reopen %s in %.0f s", device.port, delay)
    time.sleep(delay)
    try:
        device.close()
        device.open()
        log.info("Serial link to %s reestablished", device.port)
    except Exception as exc:
        log.error("Reconnect failed: %s", exc)
