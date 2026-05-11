# Wood FPM Firmware

Firmware for a 2-DOF Parallel Platform Mechanism (PPM) driven by two Maxon EPOS2
70/10 controllers on a Raspberry Pi 3B+, with a CNC touch probe on GPIO and a
LAN-accessible web UI.

## Hardware

See [`system-setup.txt`](system-setup.txt). Inverse kinematics adapted from
[`inverse-example.txt`](inverse-example.txt).

## Install (on the Pi)

```bash
sudo apt install python3-pip pigpiod
sudo systemctl enable --now pigpiod
pip3 install -r requirements.txt
# Install Maxon's EposCmd Linux library so libEposCmd.so.* is on the loader path.
```

Fill in the TBD values in [`config.yaml`](config.yaml) (PPM parameters, gear
ratios, workspace bounds) before running on real hardware.

## Run

```bash
python3 run.py --config config.yaml
```

Visit `http://<pi-ip>:5000`.

For UI development without hardware, set `driver.simulate: true` in
`config.yaml` and run on any machine.

## Tests

```bash
pytest tests/
```

## Architecture

```
┌──────────────────────────────────────────────────┐
│  Browser (LAN)                                   │
│  • Plotly.js (scan plot w/ faded history)        │
│  • Socket.IO client (live pos + probe state)     │
└────────────────────┬─────────────────────────────┘
                     │ HTTP + WebSocket
┌────────────────────▼─────────────────────────────┐
│  Flask + Flask-SocketIO  (src/web/)              │
│  • REST: /api/jog /api/home /api/scan /api/stop  │
│  • WS: state @ ~20 Hz, scan_point on contact     │
└────────────────────┬─────────────────────────────┘
                     │ thread + queue
┌────────────────────▼─────────────────────────────┐
│  Controller layer (src/core/)                    │
│  ├ MotionController  — high-level move_to(x,y)   │
│  ├ Kinematics        — inverse/forward (PPM)     │
│  ├ ProbeMonitor      — GPIO edge-detect on probe │
│  ├ ScanRunner        — orchestrates contour scan │
│  └ Limits/Bounds     — pos / vel / accel guards  │
└────────────────────┬─────────────────────────────┘
                     │ ctypes / GPIO
┌────────────────────▼─────────────────────────────┐
│  EposDriver (src/drivers/epos.py)                │
│  Wraps libEposCmd.so:                            │
│    VCS_OpenDevice, VCS_SetEnableState,           │
│    VCS_ActivateProfilePositionMode,              │
│    VCS_MoveToPosition, VCS_GetPositionIs,        │
│    VCS_FindHome (index-pulse mode), …            │
└──────────────────────────────────────────────────┘
```

The XY shown in the UI is a **user-facing workspace coordinate**, not a
Cartesian gantry. Every move command flows:

`workspace (x, y) → inverse_kinematics → (θ°, φ°) → counts → MoveToPosition`

The polling thread reverses that for live state:

`GetPositionIs → counts → (θ°, φ°) → forward_kinematics → workspace (x, y, z)`

## Modules

**[`src/core/kinematics.py`](src/core/kinematics.py)**
PPM forward & inverse, ported from `inverse-example.txt`.
`inverse_kinematics(x, y)` returns joint angles in degrees;
`forward_kinematics(θ_rad, φ_rad)` returns `(x, y, z)` relative to the home pose.
Rejects targets exceeding `theta_max`.

**[`src/core/limits.py`](src/core/limits.py)**
`MachineBounds` (workspace rectangle, raises `BoundsError`) and
`MotionLimits` (v_max, a_max).

**[`src/core/motion.py`](src/core/motion.py)**
`MotionController`: `home_all`, `move_to`, `jog`, `halt`, `stop`, `set_feed_mm_s`.
Owns the EPOS driver, runs a 20 Hz state-poll thread, broadcasts via subscriber
callbacks. Converts joint deg ↔ motor encoder counts using each axis's gear ratio.

**[`src/core/probe.py`](src/core/probe.py)**
`ProbeMonitor`: pigpio edge-detect on the configured GPIO with debounce.
Falls back to a software-only simulator when `pigpiod` is unavailable.

**[`src/core/scan.py`](src/core/scan.py)**
`ScanRunner`: for each X sample, rapid to `y_max`, slow-feed toward `y_min`
while watching the probe, halt + record on contact, retract, advance.
Emits `scan_started` / `scan_point` / `scan_complete`.

**[`src/drivers/epos.py`](src/drivers/epos.py)**
ctypes wrapper around `libEposCmd.so` (`EposDriver` + `EposAxis`), plus
`SimulatedEposDriver` / `SimulatedEposAxis` for off-hardware development.

**[`src/web/app.py`](src/web/app.py)**
Flask + Socket.IO. REST endpoints kick work onto background threads;
state and scan events stream over WS.

**[`src/web/templates/index.html`](src/web/templates/index.html) + [`static/app.js`](src/web/static/app.js)**
Single-page UI: status, limits, jog pad, scan form, Plotly plot.
Each new scan adds a trace; older traces fade with each subsequent run.

**[`run.py`](run.py)**
Entrypoint — loads `config.yaml`, wires the controllers, serves the app.

## API

| Method | Path                | Body                                                              | Notes |
|--------|---------------------|-------------------------------------------------------------------|-------|
| POST   | `/api/home`         | —                                                                 | Index-pulse home on both axes |
| POST   | `/api/jog`          | `{dx, dy}` *or* `{x, y}`                                          | Bounded by workspace, returns 400 if out of range |
| POST   | `/api/scan`         | `{x_min, x_max, y_max, y_min, n_samples}`                         | Returns `{scan_id}`; conflict 409 if a scan is running |
| POST   | `/api/scan/abort`   | —                                                                 | Cancels in-flight scan |
| POST   | `/api/stop`         | —                                                                 | E-stop: halt + disable drives |
| GET    | `/api/limits`       | —                                                                 | Reads bounds, v_max, a_max, feeds |

Socket.IO events streamed to the browser:

- `state` (~20 Hz) — `{x, y, z, theta_deg, phi_deg, probe, homed, busy, fault, last_error}`
- `scan_started` — `{scan_id, x_min, x_max, y_max, y_min, n_samples}`
- `scan_point` — `{scan_id, index, x, y}` (`y = null` means no contact on that column)
- `scan_complete` — `{scan_id}`

## Layout

```
src/
  core/        kinematics, motion, probe, scan, limits
  drivers/     EPOS2 ctypes wrapper + simulator
  web/         Flask-SocketIO server + static frontend
run.py         entrypoint
config.yaml    machine / kinematics / axes / probe / network config
tests/         pytest suite (kinematics round-trip, limits, scan sim)
```
