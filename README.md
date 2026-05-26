# Wood FPM Firmware

Firmware for a 2-DOF Parallel Platform Mechanism (PPM) driven by a Marlin
mainboard on a Raspberry Pi 3B+, with Marlin-native probing and a
LAN-accessible web UI.

## Hardware

See [`system-setup.txt`](system-setup.txt).

The Marlin firmware handles the PPM kinematics; this Pi-side firmware is
a thin shim that:
- accepts workspace XY commands from the web UI,
- forwards them as straight G-code over serial to Marlin,
- runs contour scans with Marlin's `G38.2` probe move,
- streams Marlin's reported XY position and controller state back to the browser.

The Pi never touches inverse/forward kinematics — Marlin does both.

## Install (on the Pi)

```bash
sudo apt install python3-pip

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

The user running this needs serial access — typically `sudo usermod -aG
dialout $USER` then log out/in.

Edit [`config.yaml`](config.yaml) with your workspace bounds and Marlin
settings before running on real hardware.

## Marlin configuration

A few Marlin firmware settings need to line up with this Pi-side firmware:

- The Marlin build must accept G-code in **workspace XY coordinates** and
  do its own inverse kinematics internally (e.g. the FPM model is compiled
  into Marlin's planner).
- **`G38_PROBE_TARGET`** must be enabled and the controller board's probe
  input must be configured, because scans now use `G38.2`.
- **`MIN_POS` / `MAX_POS`** for X and Y in Marlin should bracket the
  workspace rectangle in `config.yaml`'s `machine.bounds` (same units, mm).
- **`EMERGENCY_PARSER`** strongly recommended so M114, M112, M410 are
  processed even while moves are queued — the Pi polls M114 at 5 Hz for
  live state.
- Set the right **`BAUDRATE`** in Marlin and choose the matching baudrate in
  the web UI's Serial panel when you connect.
- If your Marlin build reports the workspace axes under non-standard
  letters, override `marlin.x_axis` / `y_axis` / `z_axis` in `config.yaml`.

## Run

```bash
python3 run.py --config config.yaml
```

Visit `http://<pi-ip>:5000`.

With `driver.simulate: false`, the web app starts disconnected on purpose;
pick the serial port and baudrate in the Serial panel and connect from the UI.

For UI development without hardware, set `driver.simulate: true` in
`config.yaml` and run on any machine — a `SimulatedMarlinDriver` will
respond to the G-codes in software.

## CLI

[`cli.py`](cli.py) is an interactive shell for the same motion controller
the web UI drives — handy for bring-up, manual jogging, and headless
operation. The serial connection to Marlin is opened once on startup and
closed on exit.

```text
$ python3 cli.py
Wood FPM interactive CLI. Type 'help' for commands, 'quit' to exit.
fpm> home                       # G28 on both axes
fpm> move 5 0                   # absolute workspace (x=5, y=0) mm
fpm> move abs 0 0               # absolute workspace origin (explicit)
fpm> move rel 1 0               # relative jog: +1 mm in x
fpm> status                     # current workspace XYZ from Marlin
fpm> stop                       # M410 + M84 (halt + disable steppers)
fpm> quit
```

## Tests

```bash
pytest tests/
```

## Safety limits

The Pi enforces **workspace bounds** (`machine.bounds` in `config.yaml`) —
every `move_to` rejects coordinates outside the rectangle before any
G-code is issued. Marlin's `MIN_POS` / `MAX_POS` should be the same so
the firmware enforces the limits independently.

There is no per-stepper current monitoring (the overcurrent guard that
existed in the older EPOS-based firmware doesn't have a Marlin
equivalent). Treat Marlin's configured probe input as the physical-fault
fail-safe for scans: each probe stroke is a `G38.2`, so Marlin errors if
the target is reached without a trigger.

## Architecture

```
┌──────────────────────────────────────────────────┐
│  Browser (LAN)                                   │
│  • Plotly.js (scan plot w/ faded history)        │
│  • Socket.IO client (live state)                 │
└────────────────────┬─────────────────────────────┘
                     │ HTTP + WebSocket
┌────────────────────▼─────────────────────────────┐
│  Flask + Flask-SocketIO  (src/web/)              │
│  • REST: /api/jog /api/home /api/scan /api/stop  │
│  • WS: state @ ~5 Hz, scan_point on contact      │
└────────────────────┬─────────────────────────────┘
                     │ thread + queue
┌────────────────────▼─────────────────────────────┐
│  Controller layer (src/core/)                    │
│  ├ MotionController  — workspace XY ⇄ G-code     │
│  ├ ScanRunner        — G38.2 contour scan        │
│  └ Limits/Bounds     — pos / vel / accel guards  │
└────────────────────┬─────────────────────────────┘
                     │ pyserial (G-code)
┌────────────────────▼─────────────────────────────┐
│  MarlinDriver (src/drivers/marlin.py)            │
│  Single-threaded G-code I/O over serial:         │
│    G28 (home), G1 (move), G38.2 (probe), M114,   │
│    M410 (quick stop), M84 (disable steppers),    │
│    M115 (handshake)                              │
│  + SimulatedMarlinDriver for off-hardware dev    │
└──────────────────────────────────────────────────┘
                     │ serial cable
              ┌──────▼──────┐
              │   Marlin    │  ← inverse + forward kinematics
              │ mainboard   │     run here, not on the Pi
              └─────────────┘
```

Move command flow: `workspace (x, y) → G1 X<x> Y<y> F<v×60> → Marlin → steppers`.

State refresh: `M114 → workspace (x, y, z) → broadcast to browser`.

## Modules

**[`src/core/limits.py`](src/core/limits.py)**
`MachineBounds` (workspace rectangle, raises `BoundsError`) and
`MotionLimits` (v_max, a_max).

**[`src/core/motion.py`](src/core/motion.py)**
`MotionController`: `home_all`, `move_to`, `jog`, `probe_to_x`, `halt`,
`stop`, `set_feed_mm_s`. Forwards workspace XY commands to Marlin and runs
a 5 Hz state-poll thread (M114) that converts back into the UI state.
Move completion waits for Marlin's stepper counts to settle after the
logical target is reached, so scans start their probe stroke from the
actual settled position.

**[`src/core/scan.py`](src/core/scan.py)**
`ScanRunner`: for each Y sample, rapid to `x_max`, run `G38.2` toward the
requested `probe_target_x`, record the contact X on success, retract,
advance.
Emits `scan_started` / `scan_point` / `scan_complete`.

**[`src/drivers/marlin.py`](src/drivers/marlin.py)**
G-code serial driver (`MarlinDriver`) plus `SimulatedMarlinDriver` for
off-hardware testing. A single I/O thread fed by a queue serializes
commands across callers; each `send` blocks until Marlin acks with `ok`.

**[`src/web/app.py`](src/web/app.py)**
Flask + Socket.IO. REST endpoints kick work onto background threads;
state streams over WS, scan points are appended to a CSV, and scan events
trigger CSV-backed plot refreshes in the browser.

**[`src/web/scan_history.py`](src/web/scan_history.py)**
`ScanHistoryCsvStore`: appends one CSV row per probe cycle to
`scan_history.csv` (stored next to `config.yaml` when launched via `run.py`),
serves that history back to the browser, and supports clearing it from the UI.

**[`src/web/templates/index.html`](src/web/templates/index.html) + [`static/app.js`](src/web/static/app.js)**
Single-page UI: status, limits, jog pad, scan form, Plotly plot.
Each new scan adds a trace; older traces fade with each subsequent run.

**[`run.py`](run.py)**
Entrypoint — loads `config.yaml`, wires the controllers, serves the app.

**[`src/core/kinematics.py`](src/core/kinematics.py) + [`inverse-example.txt`](inverse-example.txt)**
Reference PPM forward/inverse kinematics. Not on the firmware's runtime
path (Marlin does the kinematics) but kept as a reference / test fixture.

## API

| Method | Path                | Body                                                              | Notes |
|--------|---------------------|-------------------------------------------------------------------|-------|
| POST   | `/api/home`         | —                                                                 | Marlin G28 on both axes |
| POST   | `/api/jog`          | `{dx, dy}` *or* `{x, y}`                                          | Bounded by workspace, returns 400 if out of range |
| POST   | `/api/scan`         | `{x_max, probe_target_x, y_max, y_min, n_samples}`                | Sweeps Y across `[y_min, y_max]`, moves to `x_max`, then probes with `G38.2 X<probe_target_x>` |
| POST   | `/api/scan/abort`   | —                                                                 | Cancels in-flight scan |
| GET    | `/api/scan/history.csv` | —                                                             | Returns the persisted probe history CSV used by the plot |
| POST   | `/api/scan/history/clear` | —                                                          | Clears the persisted probe history CSV when no scan is running |
| POST   | `/api/stop`         | —                                                                 | E-stop: M410 + M84 |
| GET    | `/api/limits`       | —                                                                 | Reads bounds, v_max, a_max, feeds |

Socket.IO events streamed to the browser:

- `state` (~5 Hz) — `{x, y, z, homed, busy, fault, last_error}`
- `scan_started` — `{scan_id, probe_target_x, x_max, y_max, y_min, n_samples}`
- `scan_point` — `{scan_id, index, x, y}` (`x = null` means no contact on that row)
- `scan_complete` — `{scan_id}`

Each probe cycle is appended as one row in `scan_history.csv` with the scan ID,
row index, hit/miss flag, contact `x`, probed `y`, and the scan bounds used for
that row. The browser plot rebuilds itself from this CSV, so a page refresh
shows the persisted history rather than only the current tab's in-memory state.

## Layout

```
src/
  core/         motion, probe, scan, limits, kinematics (reference)
  drivers/      Marlin serial G-code driver + simulator
  web/          Flask-SocketIO server + static frontend
run.py          web-UI entrypoint
cli.py          command-line front-end (home, move, status, stop)
config.yaml     machine / marlin / probe / network config
tests/          pytest suite
```
