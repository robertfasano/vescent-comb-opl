# vescent-comb-opl

Python drivers for the Vescent **RUBRIComb** frequency comb and **SLICE-OPL**
offset phase lock servo, plus a read-only monitoring loop that logs every
instrument parameter to InfluxDB.

Written against:

| Instrument | API guide | Firmware |
|---|---|---|
| RUBRIComb  | Serial API Guide Rev 01 | SC 1.241, G2 1.19, AMP/OSC LD 1.22 |
| SLICE-OPL  | Serial API Guide Rev 01 | SC 1.242, OPL 1.27 |

## Layout

| File | Contents |
|---|---|
| `vescent_serial.py` | Shared transport: serial I/O, echo handling, response parsers, `Param` table type, read-only enforcement, Influx sinks, monitor loop |
| `rubricomb.py` | `RubriComb` driver + `MONITOR_PARAMS` (30 parameters) |
| `slice_opl.py` | `SliceOPL` driver + `MONITOR_PARAMS` (59 parameters) |
| `main.py` | Read-only entry point: reads `config.yaml`, connects to every configured box, logs to Influx |
| `config.yaml` | Instrument config: serial defaults, Influx settings, one RUBRIComb, any number of SLICE-OPLs |
| `test_mock.py` | Offline test against simulated instruments (no hardware needed) |

Both instruments speak the same protocol — 8N1, no flow control, CR-terminated
case-insensitive ASCII — so everything except the command sets is shared.

## Install

```bash
pip install -r requirements.txt   # pyserial, influxdb-client, PyYAML
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

rubricomb:
  port: COM7
  measurement: rubricomb

slice_opls:
  - name: opl-556          # Influx measurement name -- must be unique
    port: COM5
  - name: opl-reprate
    port: COM8
    # baud: 9600            # per-entry override of the 'serial' defaults
```

Either the `rubricomb` section or `slice_opls` list may be omitted if you don't
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

with SliceOPL("/dev/ttyUSB1") as opl:          # read-only by default
    print(opl.idn(), opl.error_signal(), opl.lock_range())

with SliceOPL("/dev/ttyUSB1", read_only=False) as opl:
    opl.set_pll_gain(-10.0)
```

## Read-only by default

Devices refuse to transmit state-changing commands unless constructed with
`read_only=False`. The check runs before any byte hits the wire and is an
explicit allowlist, not a guess at punctuation — necessary because the SLICE-OPL
API breaks the `?` convention in both directions:

* `READVOLT` **reads** an ADC channel with no `?` → declared read-safe.
* `NOCP`, `DDSAUTO`, `_SELFCAL`, `_FACTORY` **change state** with no `?` →
  never treated as reads.

`main.py` hardcodes `read_only=True`, invokes no startup/shutdown sequence, and
opens ports with DTR/RTS deasserted so that connecting cannot pulse a reset into
the USB front end. Running it twice, or killing it mid-sweep, leaves the
hardware untouched.

## Monitoring

Each driver's `MONITOR_PARAMS` tuple is the full list of readable parameters:
Influx field name, query string, parser, subsystem, unit. Comment out any line
to stop logging that parameter while keeping it documented. `read_all()` sweeps
the table and returns `(values, failures)` — a timeout or malformed reply drops
that one field and logs a warning instead of aborting the sweep. A
`read_failures` count goes into every point so a degrading link is visible on
the dashboard.

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
