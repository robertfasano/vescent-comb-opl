#!/usr/bin/env python3
"""
api.py -- FastAPI HTTP interface for on-demand instrument reads/writes.

Endpoints
---------
    GET /                              index of every measurement/field
    GET /{measurement}/{field}         read <field> from <measurement> now
    GET /{measurement}/{field}/{value} write <value>, then re-read <field>

Every GET triggers an immediate hardware read or write -- nothing here is
cached or served from the periodic monitor() sweep. It runs alongside that
sweep, not instead of it: VescentSerialDevice and SliceFPGA both serialize
their I/O with an internal lock, so a request landing mid-sweep queues
rather than corrupting the exchange.

Write path
----------
A field only accepts writes if both hold:
  1. its Param has readonly=False, and
  2. that Param has a setter wired up (an existing, already-tested driver
     method -- never a raw command string built from the URL).

Right now only rubricomb.py's cav_temp has readonly=False plus a setter;
everything else 405s.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Sequence

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse

from vescent_serial import MonitoredDevice, VescentError

log = logging.getLogger("vescent.api")


def _device_or_404(by_measurement: Dict[str, Any], measurement: str) -> Any:
    device = by_measurement.get(measurement)
    if device is None:
        raise HTTPException(
            404, f"no device named {measurement!r} is running "
                 f"(available: {sorted(by_measurement)})")
    return device


def _param_or_404(device: Any, measurement: str, field: str):
    for p in getattr(device, "MONITOR_PARAMS", ()):
        if p.key == field:
            return p
    known = sorted(p.key for p in getattr(device, "MONITOR_PARAMS", ()))
    raise HTTPException(
        404, f"{field!r} is not a known field of {measurement!r} (available: {known})")


def create_app(devices: Sequence[MonitoredDevice]) -> FastAPI:
    """Build the FastAPI app bound to *devices* -- the same MonitoredDevice
    list main.py's monitor() loop polls, so reads/writes here hit the exact
    same live connections, not a separate handle to the same port."""
    by_measurement: Dict[str, Any] = {d.measurement: d.device for d in devices}

    app = FastAPI(
        title="vescent-comb-opl",
        description="On-demand reads/writes against the currently-connected instruments.",
    )

    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        parts = ["<h1>vescent-comb-opl</h1>"]
        for measurement, device in sorted(by_measurement.items()):
            parts.append(f"<h2>{measurement}</h2><ul>")
            params = sorted(getattr(device, "MONITOR_PARAMS", ()), key=lambda p: p.key)
            for p in params:
                writable = not p.readonly and p.setter is not None
                tag = " &mdash; <b>writable</b>" if writable else ""
                unit = f" [{p.unit}]" if p.unit else ""
                parts.append(
                    f'<li><a href="/{measurement}/{p.key}">{measurement}/{p.key}</a>'
                    f"{unit}{tag}</li>")
            parts.append("</ul>")
        return "\n".join(parts)

    @app.get("/{measurement}/{field}")
    def read(measurement: str, field: str) -> Dict[str, Any]:
        device = _device_or_404(by_measurement, measurement)
        param = _param_or_404(device, measurement, field)
        try:
            value = device.query(param.command, param.parser)
        except VescentError as exc:
            raise HTTPException(502, f"hardware read failed: {exc}") from exc
        return {"measurement": measurement, "field": field, "value": value,
                "unit": param.unit}

    @app.get("/{measurement}/{field}/{value}")
    def write(measurement: str, field: str, value: str) -> Dict[str, Any]:
        device = _device_or_404(by_measurement, measurement)
        param = _param_or_404(device, measurement, field)
        if param.readonly or param.setter is None:
            reason = "marked readonly" if param.readonly else "no write handler wired up yet"
            raise HTTPException(405, f"{field!r} is not writable ({reason})")
        try:
            parsed = param.parser(value)
        except Exception as exc:
            raise HTTPException(400, f"could not parse {value!r} for {field!r}: {exc}") from exc
        try:
            param.setter(device, parsed)
            new_value = device.query(param.command, param.parser)
        except ValueError as exc:
            raise HTTPException(400, str(exc)) from exc
        except VescentError as exc:
            raise HTTPException(502, f"hardware write failed: {exc}") from exc
        return {"measurement": measurement, "field": field, "value": new_value,
                "unit": param.unit, "written": True}

    return app
