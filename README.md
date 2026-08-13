# vescent-comb-opl

Python drivers for the Vescent **RUBRIComb** frequency comb, **SLICE-FPGA**
comb-locking electronics, and **SLICE-OPL** offset phase lock servo, plus a
monitoring loop that logs every instrument parameter to InfluxDB and an
optional HTTP API for on-demand reads (and, for the small set of fields
opted into it, writes).

Written against:

| Instrument | API guide | Firmware |
|---|---|---|
| RUBRIComb  | Serial API Guide Rev 01 | SC 1.241, G2 1.19, AMP/OSC LD 1.22 |
| SLICE-OPL  | Serial API Guide Rev 01 | SC 1.242, OPL 1.27 |
| SLICE-FPGA | undocumented -- `slice_fpga.MONITOR_PARAMS` reflects only what's been confirmed working from a PuTTY session against the vendor software's Telnet/IPC bridge | n/a |

## Layout

| File | Contents |
|---|---|
| `vescent_serial.py` | Shared serial transport: I/O, echo handling, response parsers, `Param` table type, Influx sinks, monitor loop |
| `rubricomb.py` | `RubriComb` driver + `MONITOR_PARAMS` (30 parameters) |
| `slice_opl.py` | `SliceOPL` driver + `MONITOR_PARAMS` (59 parameters) |
| `slice_fpga.py` | `SliceFPGA` Telnet/socket driver + `MONITOR_PARAMS`, for the SLICE-FPGA IPC bridge |
| `api.py` | FastAPI app: on-demand `GET` reads/writes against the live devices, by measurement/field name |
| `main.py` | Entry point: reads `config.yaml`, connects to every configured box, logs to Influx, starts the API |
| `config.yaml` | Instrument config: serial defaults, Influx settings, API settings, one RUBRIComb, one SLICE-FPGA, any number of SLICE-OPLs |
| `test_mock.py` | Offline test against simulated instruments (no hardware needed) |

RUBRIComb and SLICE-OPL speak the same serial protocol — 8N1, no flow control,
CR-terminated case-insensitive ASCII — so everything except the command sets
is shared via `vescent_serial.py`. SLICE-FPGA is different: it isn't reachable
over a COM port at all, only through the vendor control software's Telnet/IPC
bridge (`127.0.0.1:65432` by default), so `slice_fpga.py` implements its own
socket transport but mirrors the same `open()`/`close()`/`query()`/`read_all()`
interface so it drops into the same monitoring loop.

## Install

```bash
pip install -r requirements.txt   # pyserial, influxdb-client, PyYAML, fastapi, uvicorn
```

Drop `influx_example.py` (the `write_point` helper) alongside these files and it
is imported automatically; without it an equivalent fallback is used.

`INFLUX_URL` and `INFLUX_ORG` are read from the environment (falling back to
`http://localhost:8086` / `yblab` if unset) rather than from the config file,
since they tend to be per-machine rather than per-instrument-set. `INFLUX_TOKEN`
works the same way and can also be set via `influx.token` in the config.
`INFLUX_BUCKET` is likewise honored as an env var but is normally set via
`influx.bucket` in the config, below.

## Configuration

`main.py` takes no per-instrument flags -- ports, measurement names, and Influx
settings all live in `config.yaml`, so a rig with several SLICE-OPL channels
doesn't need a wall of command-line arguments:

```yaml
serial:
  baud: 115200
  timeout: 2.0
  assert_dtr: false

influx:
  bucket: vescent-demo
  mode: per-field

polling:
  interval: 10.0

api:
  enabled: true          # only takes effect in continuous mode, not --once
  host: 127.0.0.1
  port: 1993

rubricomb:
  port: COM7
  measurement: rubricomb

slice_fpga:
  host: 127.0.0.1        # the vendor software's Telnet/IPC bridge, not a COM port
  port: 65432
  measurement: slice_fpga

slice_opls:
  - name: opl-556          # Influx measurement name -- must be unique
    port: COM5
  - name: opl-reprate
    port: COM8
    # baud: 9600            # per-entry override of the 'serial' defaults
```

Any of `rubricomb`, `slice_fpga`, or `slice_opls` may be omitted if you don't
have that instrument. Set `INFLUX_URL` / `INFLUX_ORG` (and `INFLUX_TOKEN`, if
you'd rather not put it in the config) in your shell profile or service
environment:

```bash
export INFLUX_URL=http://localhost:8086
export INFLUX_ORG=yblab
```

## Usage

```bash
# one snapshot of every configured instrument, printed, nothing written to Influx
python main.py --once --dry-run

# continuous logging at the interval set in the config
python main.py -v

# point at a config file that isn't ./config.yaml
python main.py --config /path/to/other_config.yaml

# offline tests
python test_mock.py
```

Interactive use:

```python
from slice_opl import SliceOPL

with SliceOPL("/dev/ttyUSB1") as opl:
    print(opl.idn(), opl.error_signal(), opl.lock_range())
    opl.set_pll_gain(-10.0)   # sends immediately -- see "What's writable" below
```

## HTTP API

`python main.py -v` (continuous mode, not `--once`) also starts a small
FastAPI server -- `config.yaml`'s `api:` section, `127.0.0.1:1993` by
default -- so a field can be read (or, for the handful opted into it,
written) straight from a browser instead of a Python session:

```
GET /                              index page: every measurement/field, clickable
GET /{measurement}/{field}         read <field> from <measurement> right now
GET /{measurement}/{field}/{value} write <value> to <field>, then re-read it
```

`{measurement}` is whatever name that device was given in `config.yaml`
(`rubricomb`, `opl-556`, ...) and `{field}` is the `Param.key` from that
driver's `MONITOR_PARAMS` -- the same names InfluxDB fields use. Every GET
hits the instrument directly; nothing here is cached or served from the
periodic sweep.

```bash
curl http://localhost:1993/rubricomb/cav_temp
# {"measurement":"rubricomb","field":"cav_temp","value":24.21,"unit":"degC"}

curl http://localhost:1993/opl-reprate/beat_note_divided
```

Or just type the URL into a browser's address bar -- that's the point of the
GET-with-a-trailing-value write form (`.../cav_temp/24.5`) too: no client, no
form, no JS, just a link.

**A field is writable only if its `Param` has `readonly=False` *and* a
`setter` wired up** -- today that's exactly one field, `rubricomb.py`'s
`cav_temp`, wired to `set_cavity_temperature_setpoint()` (see
[What's writable](#whats-writable)). If a field passes that check, the
write goes straight to the instrument. Most `MONITOR_PARAMS` entries are
physical measurements (temperatures, SNR, ADC voltages, beat note
readings, ...) with no corresponding hardware *setter* at all, so they
405 rather than doing something undefined.

Response codes: `200` on success (write responses include the freshly
re-read value, which for something like `cav_temp` won't jump to match the
new setpoint instantly -- it's a measured temperature responding to a PID
loop, not the setpoint echoed back), `404` for an unknown measurement or
field, `405` for a field that isn't writable, `400` for a value that doesn't
parse, `502` if the hardware itself errors or times out.

## What's writable

`RubriComb`, `SliceOPL`, and `SliceFPGA` methods send whatever command they
build as soon as they're called -- `comb.set_master_mode(...)`,
`opl.reset()`, anything. What automatically gets called is narrow:

* **`main.py`** only calls `read_all()` against `MONITOR_PARAMS`, plus --
  through the HTTP API -- a `Param.setter` for fields marked
  `readonly=False`.
* **The HTTP API** checks `Param.readonly`/`Param.setter` per field (see
  [HTTP API](#http-api)) before calling a setter: `readonly=True` (every
  field except `cav_temp`) or no `setter` wired up both 405.

Two SLICE-OPL API quirks worth knowing if you're calling driver methods
directly, since the `?` suffix isn't a reliable read/write signal on that
instrument:

* `READVOLT` **reads** an ADC channel with no `?`.
* `NOCP`, `DDSAUTO`, `_SELFCAL`, `_FACTORY` **change state** with no `?`.

`main.py` opens serial ports with DTR/RTS deasserted, so connecting can't
pulse a reset into the USB front end, and never invokes a
startup/shutdown/laser-enable sequence on its own.

## Monitoring

Each driver's `MONITOR_PARAMS` tuple lists every field the driver knows how
to read, not just the ones actively logged -- every entry carries two
independent flags:

* `poll` -- included in the periodic sweep that gets logged to Influx.
  `poll=False` keeps the field in the table, documented and reachable by
  the HTTP API on demand, without it showing up in `monitor()`'s Influx
  writes every cycle. Toggle it to change what gets logged.
* `readonly` -- whether the HTTP API will ever write to this field. `True` for
  everything except `rubricomb.py`'s `cav_temp`, the one field currently wired
  up to test write support against (see [What's writable](#whats-writable)).

`read_all()` sweeps `poll=True` entries by default (pass an explicit `params=`
to bypass that, e.g. for a one-off full read) and returns `(values, failures)`
— a timeout or malformed reply drops that one field and logs a warning instead
of aborting the sweep. A `read_failures` count goes into every point so a
degrading link is visible on the dashboard.

A dead connection (port unplugged/access denied, or the SLICE-FPGA socket
dropped) is treated differently from a bad reply: it aborts the sweep instead
of being recorded as 59 identical per-field failures, which is what lets
`monitor()`'s reconnect logic (close + reopen, retried every 5 s) actually
kick in instead of the device silently going dark for the rest of the run.

Derived fields are computed per instrument: the RUBRIComb decodes its three
`*ERROR?` bitmasks into `cavity_ok` / `oscillator_ok` / `amplifier_ok`, and the
SLICE-OPL flags `lock_ok` when the servo is engaged and |error signal| sits
inside `LOCKRNG?`.

### Influx notes

* `influx.mode: per-field` (default) uses the attached `write_point`, opening a
  client per field: ~30 connections per RUBRIComb sweep, ~59 per SLICE-OPL
  sweep. Fine at 10 s on localhost.
* `influx.mode: batched` keeps one client alive and writes one point per device
  per sweep, all fields sharing a timestamp. Use it below ~10 s cadence; it also
  makes Flux joins across channels exact.
* Numeric values are cast to `float` before writing, since Influx rejects a
  point whose field type conflicts with what the bucket already holds
  (`CAVCURRENT?` → `.654` vs. an error code → `49152`).

## API gotchas encoded here

* Reply shapes vary: bare value (`24.21`), echo-plus-value (`MSTRCTL? 0`,
  `#SCBKLT? 5`), and pure echo (`#SAVESETTINGS`). All three are handled.
* RUBRIComb `CAVTERROR?` reports **mK** while `OSCTERROR?`/`AMPTERROR?` report
  **°C**; current setpoints are **A** but readbacks are **mA**. Field names carry
  units.
* SLICE-OPL `PLLGAIN` is documented as dB in the description and Hz in the
  argument list; treated as dB.
* `READBN?` returns the *divided* beat note, and Rev 01 does not state whether
  the reading sits after N1 alone or N1·N2 — so no reconstructed carrier is
  logged. Set `SliceOPL.ESTIMATE_BEAT_NOTE = True` once you've checked it
  against a counter.
* `OSCINTERLK?`/`AMPINTERLK?` are listed as taking no parameters but the
  examples show a channel argument; sent bare. Add the argument to the `Param`
  command string if your firmware wants it.
* `_FACTORY` is deliberately not implemented — it wipes system calibration.
* SLICE-FPGA requires the vendor control software to be running with its
  Telnet/IPC bridge listening (default `127.0.0.1:65432`) -- `slice_fpga.py`
  is a client to that bridge, not a direct connection to the FPGA hardware.
  Its `MONITOR_PARAMS` units are marked `dB` as placeholders pending
  confirmation against the actual firmware documentation.
