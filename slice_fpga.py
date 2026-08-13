#!/usr/bin/env python3
"""
slice_fpga.py -- Telnet driver for the Vescent SLICE-FPGA.

Usage
-----
    # interactive, behaves like the PuTTY window
    python slice_fpga.py

    # one-shot query
    python slice_fpga.py --command "creffreq?"

    # from code
    from slice_fpga import SliceFPGA
    with SliceFPGA() as fpga:
        print(fpga.query("creffreq?"))
"""

from __future__ import annotations

import argparse
import logging
import socket
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from vescent_serial import (
    NoResponseError,
    Param,
    VescentError,
    parse_float,
    parse_str,
    strip_echo,
)

log = logging.getLogger("vescent.slice_fpga")

SliceFPGAError = VescentError


class ConnectionLostError(VescentError):
    """The TCP connection to the SLICE-FPGA bridge is gone.

    Distinct from NoResponseError/ResponseParseError (a single bad reply):
    once this fires every remaining query in the sweep would fail identically,
    so read_all() lets it propagate instead of recording it as a per-field
    failure -- that's what tells the monitor loop to reconnect rather than
    silently accumulating failures forever.
    """


# ---------------------------------------------------------------------------
# Telnet protocol bytes (RFC 854)
# ---------------------------------------------------------------------------

IAC = 255   # interpret as command
DONT = 254
DO = 253
WONT = 252
WILL = 251
SB = 250    # subnegotiation begin
SE = 240    # subnegotiation end

_NEGOTIATION = (DO, DONT, WILL, WONT)


# ---------------------------------------------------------------------------
# Parameter table -- add entries here as the API is mapped out.
# Same structure as rubricomb.py / slice_opl.py: comment out what you do not
# want logged, keep the rest for reference.
# ---------------------------------------------------------------------------

MONITOR_PARAMS: Tuple[Param, ...] = (
    # Confirmed working from the PuTTY session; units not yet known.
    Param("fceo_output_voltage", "coutvolt?", parse_float, "fceo", "V"),
    Param("fceo_snr", "csnr?", parse_float, "fceo", "dB"),
    Param("fceo_phase_noise", "cphnoisestd?", parse_float, "fceo", "dB"),
    Param("fceo_peak_freq", "cpeakfreq?", parse_float, "fceo", "dB"),
    Param("fceo_ref_freq", "creffreq?", parse_float, "fceo", "dB"),

    Param("fopt_output_voltage", "ooutvolt?", parse_float, "fopt", "V"),
    Param("fopt_snr", "osnr?", parse_float, "fopt", "dB"),
    Param("fopt_phase_noise", "ophnoisestd?", parse_float, "fopt", "dB"),
    Param("fopt_peak_freq", "opeakfreq?", parse_float, "fopt", "dB"),
    Param("fopt_ref_freq", "oreffreq?", parse_float, "fopt", "dB"),
)


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

class SliceFPGA:
    """Telnet/socket driver for the SLICE-FPGA IPC bridge.

    Mirrors the interface of the serial drivers -- open/close, context manager,
    query(), command(), read_all() -- so it can be dropped into the same
    monitoring loop.

    Example
    -------
        with SliceFPGA() as fpga:                 # 127.0.0.1:65432
            print(fpga.query("creffreq?"))        # -> '21.4'
            print(fpga.query("creffreq?", parse_float))   # -> 21.4
    """

    DEFAULT_HOST = "127.0.0.1"
    DEFAULT_PORT = 65432

    #: PuTTY's "Return key sends Telnet New Line" maps to CR LF.
    TERMINATOR = b"\r\n"

    MONITOR_PARAMS = MONITOR_PARAMS

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        timeout: float = 2.0,
        connect_timeout: float = 5.0,
        inter_command_delay: float = 0.02,
        terminator: bytes = TERMINATOR,
        negotiate_telnet: bool = True,
        open_on_init: bool = True,
        transport: Any = None,
    ) -> None:
        """
        negotiate_telnet: strip and answer IAC sequences. Harmless against a
            raw socket server (which never sends them); set False only if the
            payload itself can contain 0xFF bytes.
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.connect_timeout = connect_timeout
        self.inter_command_delay = inter_command_delay
        self.terminator = terminator
        self.negotiate_telnet = negotiate_telnet
        self._sock: Optional[socket.socket] = transport
        self._buf = bytearray()      # decoded payload awaiting a line break
        self._iac_state: List[int] = []
        # An API request and a periodic sweep can land on the same socket at
        # once; this serializes them the same way VescentSerialDevice does.
        self._lock = threading.RLock()
        if transport is None and open_on_init:
            self.open()

    # -- connection management ----------------------------------------------

    @property
    def address(self) -> str:
        return f"{self.host}:{self.port}"

    @property
    def is_open(self) -> bool:
        return self._sock is not None

    def open(self) -> None:
        with self._lock:
            if self._sock is not None:
                return
            log.info("Connecting to %s", self.address)
            try:
                sock = socket.create_connection((self.host, self.port),
                                                timeout=self.connect_timeout)
            except OSError as exc:
                raise VescentError(
                    f"could not connect to {self.address}: {exc}. Is the SLICE-FPGA "
                    f"software running and its IPC bridge listening?"
                ) from exc
            sock.settimeout(self.timeout)
            # Commands are tiny; Nagle would add up to 40 ms of latency per exchange.
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._sock = sock
            self._buf.clear()
            self._iac_state.clear()
            # Passive negotiation: send nothing, and drain any banner or option
            # negotiation the server opens with.
            self.drain(settle=0.2)

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    self._sock.close()
                except OSError:  # pragma: no cover
                    pass
                self._sock = None

    def reconnect(self) -> None:
        self.close()
        self.open()

    def __enter__(self) -> "SliceFPGA":
        self.open()
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    # -- telnet layer --------------------------------------------------------

    def _handle_iac(self, data: bytes) -> bytes:
        """Strip IAC sequences from *data* and answer negotiation passively.

        Passive means we never propose options; anything the server proposes is
        refused (DO -> WONT, WILL -> DONT), which keeps the stream clean ASCII.
        A server that speaks plain TCP never triggers any of this.
        """
        if not self.negotiate_telnet or IAC not in data:
            return data
        out = bytearray()
        replies = bytearray()
        state = self._iac_state
        for byte in data:
            if not state:
                if byte == IAC:
                    state.append(IAC)
                else:
                    out.append(byte)
                continue
            if state[-1] == SB:                    # inside subnegotiation
                if byte == IAC:
                    state.append(IAC)
                continue
            if state[-1] == IAC and len(state) > 1 and state[0] == SB:
                if byte == SE:                     # end of subnegotiation
                    state.clear()
                else:
                    state.pop()                    # escaped 0xFF inside SB
                continue
            if state[-1] == IAC:
                if byte == IAC:                    # escaped literal 0xFF
                    out.append(IAC)
                    state.clear()
                elif byte in _NEGOTIATION:
                    state.append(byte)
                elif byte == SB:
                    state.clear()
                    state.append(SB)
                else:                              # standalone command
                    state.clear()
                continue
            verb, option = state[-1], byte         # negotiation verb + option
            if verb == DO:
                replies += bytes((IAC, WONT, option))
            elif verb == WILL:
                replies += bytes((IAC, DONT, option))
            # DONT / WONT need no answer.
            state.clear()
        if replies and self._sock is not None:
            log.debug("Refusing telnet options: %s", replies.hex())
            try:
                self._sock.sendall(bytes(replies))
            except OSError as exc:  # pragma: no cover
                log.warning("Failed to answer telnet negotiation: %s", exc)
        return bytes(out)

    # -- low-level I/O -------------------------------------------------------

    def _recv(self) -> bytes:
        """One recv() worth of payload, IAC removed. b'' on timeout."""
        if self._sock is None:
            raise ConnectionLostError("not connected")
        try:
            chunk = self._sock.recv(4096)
        except socket.timeout:
            return b""
        except OSError as exc:
            raise ConnectionLostError(f"connection to {self.address} failed: {exc}") from exc
        if chunk == b"":
            self.close()
            raise ConnectionLostError(f"{self.address} closed the connection")
        return self._handle_iac(chunk)

    def _read_line(self, timeout: Optional[float] = None) -> str:
        """Read one CR/LF-terminated line. Returns '' on timeout."""
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while True:
            for sep in (b"\r\n", b"\n", b"\r"):
                idx = self._buf.find(sep)
                if idx != -1:
                    line = bytes(self._buf[:idx])
                    del self._buf[:idx + len(sep)]
                    text = line.decode("ascii", errors="replace").strip()
                    if text:
                        return text
                    break  # blank line, keep looking
            else:
                pass
            if time.monotonic() >= deadline:
                # Some servers reply without a trailing newline; take what's there.
                if self._buf.strip():
                    text = bytes(self._buf).decode("ascii", errors="replace").strip()
                    self._buf.clear()
                    return text
                return ""
            if not self._buf or not any(s in self._buf for s in (b"\r", b"\n")):
                self._buf += self._recv()

    def drain(self, settle: float = 0.2) -> List[str]:
        """Read and return whatever is already waiting (banner, stale replies)."""
        lines: List[str] = []
        deadline = time.monotonic() + settle
        while time.monotonic() < deadline:
            chunk = self._recv()
            if chunk:
                self._buf += chunk
                deadline = time.monotonic() + settle
        while True:
            line = self._read_line(timeout=0.0)
            if not line:
                break
            lines.append(line)
        if lines:
            log.debug("Drained: %s", lines)
        return lines

    # -- transactions --------------------------------------------------------

    def _transact(self, command: str, expect_reply: bool = True,
                  max_lines: int = 4, allow_echo_reply: bool = False) -> str:
        """Send one command and return the first non-echo reply line."""
        with self._lock:
            if self._sock is None:
                raise ConnectionLostError("not connected")
            payload = command.strip().encode("ascii") + self.terminator
            log.debug("TX: %s", command.strip())
            self._buf.clear()          # discard anything stale before asking
            try:
                self._sock.sendall(payload)
            except OSError as exc:
                raise ConnectionLostError(f"send to {self.address} failed: {exc}") from exc
            if not expect_reply:
                time.sleep(self.inter_command_delay)
                return ""
            last_echo: Optional[str] = None
            for _ in range(max_lines):
                raw = self._read_line()
                if not raw:
                    break
                log.debug("RX: %s", raw)
                stripped = strip_echo(raw, command)
                if stripped is not None:
                    time.sleep(self.inter_command_delay)
                    return stripped
                last_echo = raw
            time.sleep(self.inter_command_delay)
            if allow_echo_reply and last_echo is not None:
                return last_echo
            raise NoResponseError(f"no reply to {command!r} from {self.address}")

    def query(self, command: str,
              parser: Callable[[str], Any] = parse_str) -> Any:
        """Send a command and return the parsed reply.

            fpga.query("creffreq?")               -> '21.4'
            fpga.query("creffreq?", parse_float)  -> 21.4
        """
        return parser(self._transact(command))

    def command(self, command: str, expect_reply: bool = True) -> str:
        """Send a command, returning the reply (or echo) as text.

        Use this for anything that is not obviously a query; with an
        undocumented API the distinction is yours to make.
        """
        return self._transact(command, expect_reply=expect_reply,
                              allow_echo_reply=True)

    def query_lines(self, command: str, settle: float = 0.3,
                    max_lines: int = 200) -> List[str]:
        """Send a command and collect every line it produces.

        For multi-line replies -- help text, status dumps, listings -- where the
        single-line query() would return only the first line. Useful while
        mapping out the API.
        """
        if self._sock is None:
            raise ConnectionLostError("not connected")
        self._buf.clear()
        self._sock.sendall(command.strip().encode("ascii") + self.terminator)
        lines: List[str] = []
        deadline = time.monotonic() + settle
        while time.monotonic() < deadline and len(lines) < max_lines:
            line = self._read_line(timeout=max(0.0, deadline - time.monotonic()))
            if line:
                if strip_echo(line, command) is not None:
                    lines.append(line)
                deadline = time.monotonic() + settle
        return lines

    # -- monitoring integration ---------------------------------------------

    def idn(self) -> str:
        """No documented ID string for the FPGA IPC bridge (undocumented API,
        no confirmed *IDN?-equivalent) -- report the endpoint instead, so the
        startup identification pass has something to print without guessing
        at a command that might not be a query."""
        return f"SLICE-FPGA @ {self.address}"

    def derived_fields(self, values: Dict[str, Any]) -> Dict[str, Any]:
        """Hook for computed fields once the API is understood."""
        return {}

    def read_all(
        self,
        params: Optional[Sequence[Param]] = None,
        include_derived: bool = True,
    ) -> Tuple[Dict[str, Any], Dict[str, str]]:
        """Poll every parameter in the table once.

        With no explicit *params*, sweeps only the poll=True entries of
        MONITOR_PARAMS -- same convention as the serial drivers' read_all().

        A single bad *reply* (timeout, garbled line) is recorded as a failure,
        not an aborted sweep -- same policy as the serial drivers. A
        ConnectionLostError is different: the socket itself is gone, every
        remaining query would fail identically, so it propagates instead of
        being folded into *failures* -- that's what lets the monitor loop's
        reconnect logic take over instead of this looping forever.
        """
        if params is None:
            params = tuple(p for p in self.MONITOR_PARAMS if p.poll)
        values: Dict[str, Any] = {}
        failures: Dict[str, str] = {}
        for p in params:
            try:
                values[p.key] = self.query(p.command, p.parser)
            except ConnectionLostError:
                raise
            except (VescentError, ValueError) as exc:
                failures[p.key] = f"{type(exc).__name__}: {exc}"
                log.warning("Failed to read %s (%s): %s", p.key, p.command, exc)
        if include_derived and values:
            try:
                values.update(self.derived_fields(values))
            except Exception as exc:
                log.warning("Derived field computation failed: %s", exc)
        return values, failures


# ---------------------------------------------------------------------------
# Interactive prompt
# ---------------------------------------------------------------------------

def interactive(fpga: SliceFPGA) -> None:
    """A PuTTY-like prompt: type a command, see the reply.

    Blank line re-reads without sending (for unsolicited output), ':lines <cmd>'
    collects a multi-line reply, and Ctrl-D or 'exit' quits.
    """
    print(f"Connected to {fpga.address}. Ctrl-D or 'exit' to quit; "
          f"':lines <cmd>' for multi-line replies.")
    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if line.lower() in ("exit", "quit"):
            return
        if not line:
            for extra in fpga.drain(settle=0.3):
                print(extra)
            continue
        try:
            if line.startswith(":lines "):
                for reply in fpga.query_lines(line[len(":lines "):]):
                    print(reply)
            else:
                print(fpga.command(line))
        except NoResponseError:
            print("(no reply)")
        except VescentError as exc:
            print(f"error: {exc}")
            if not fpga.is_open:
                return


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Telnet driver for the SLICE-FPGA")
    p.add_argument("--host", default=SliceFPGA.DEFAULT_HOST)
    p.add_argument("--port", type=int, default=SliceFPGA.DEFAULT_PORT)
    p.add_argument("--timeout", type=float, default=2.0, help="read timeout [s]")
    p.add_argument("--command", "-c", help="send one command and exit")
    p.add_argument("--lines", action="store_true",
                   help="with --command, collect a multi-line reply")
    p.add_argument("--no-telnet", action="store_true",
                   help="disable IAC handling (treat as a raw socket)")
    p.add_argument("-v", "--verbose", action="count", default=0)
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose >= 2 else
              logging.INFO if args.verbose == 1 else logging.WARNING,
        format="%(asctime)s %(levelname)-7s %(message)s",
    )
    try:
        fpga = SliceFPGA(args.host, args.port, timeout=args.timeout,
                         negotiate_telnet=not args.no_telnet)
    except VescentError as exc:
        print(exc, file=sys.stderr)
        return 1
    try:
        if args.command:
            if args.lines:
                for line in fpga.query_lines(args.command):
                    print(line)
            else:
                print(fpga.command(args.command))
        else:
            interactive(fpga)
    except VescentError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        fpga.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())