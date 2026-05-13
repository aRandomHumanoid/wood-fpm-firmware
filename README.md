# Wood FPM Firmware

Firmware for a 2-DOF Parallel Platform Mechanism (PPM) driven by two Maxon EPOS2
70/10 controllers on a Raspberry Pi 3B+, with a CNC touch probe on GPIO and a
LAN-accessible web UI.

## Hardware

Note that a blinking green light means not enabled, 
a solid green line means enabled, 
and a solid red light means error.

See [`system-setup.txt`](system-setup.txt). Inverse kinematics adapted from
[`inverse-example.txt`](inverse-example.txt).

## Install (on the Pi)

```bash
sudo apt install python3-pip pigpiod
sudo systemctl enable --now pigpiod

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Then install Maxon's **EPOS Command Library for Linux** so `libEposCmd.so` is
on the loader path. The vendor archive ships an installer that drops a
versioned `.so` into `/opt/EposCmdLib_<version>/lib/v8/` (on aarch64) and
symlinks it at `/usr/lib/libEposCmd.so`:

```bash
cd EPOS_Linux_Library
sudo bash install.sh
```

The installer also adds an FTDI udev rule so the EPOS2 USB devices are
accessible without sudo. Unplug/replug the USB cables once after install.

Fill in the TBD values in [`config.yaml`](config.yaml) (PPM parameters, gear
ratios, workspace bounds) before running on real hardware.

## Run

```bash
python3 run.py --config config.yaml
```

Visit `http://<pi-ip>:5000`.

For UI development without hardware, set `driver.simulate: true` in
`config.yaml` and run on any machine.

## CLI

[`cli.py`](cli.py) is an interactive shell for the same motion controller
the web UI drives — handy for bring-up, manual jogging, and headless
operation. Both EPOS connections are opened once on startup and reused
across commands; type `quit` (or Ctrl-D) to disable the drives and exit.

```text
$ python3 cli.py
Wood FPM interactive CLI. Type 'help' for commands, 'quit' to exit.
fpm> home                       # drive both joints into their hard stops
fpm> move 5 0                   # absolute workspace (x=5, y=0) mm
fpm> move abs 0 0               # absolute workspace origin (explicit)
fpm> move rel 1 0               # relative jog: +1 mm in x
fpm> rotate theta -45           # relative: rotate theta by -45°
fpm> rotate abs theta -45       # absolute: drive theta to joint angle -45°
fpm> rotate phi   -30
fpm> status                     # joint angles + xyz
fpm> stop                       # E-stop: halt + disable drives
fpm> enable                     # re-engage after stop / fault
fpm> quit
```

`move` defaults to absolute, `rotate` defaults to relative — pass `abs` or
`rel` as the first keyword to override.

Type `help` to list commands, `help <cmd>` for the docstring of a single
command. Arrow keys give command history within a session.

All commands honor the same safety guards as the web UI: workspace bounds,
post-homing software position limits, and the overcurrent ceiling (see
[Safety limits](#safety-limits)). Ctrl-C during a long-running command
(home, move) halts motion and returns to the prompt instead of exiting.

## Tests

Unit tests (pure Python, no hardware required):

```bash
pytest tests/
```

[`config_tests/`](config_tests/) holds **hardware bring-up scripts** that
talk to the EPOS2 controllers directly. These are not run by `pytest`;
they're meant to be invoked by hand during commissioning to verify a
single piece of the hardware path before trusting the full firmware:

- [`config_tests/motor_test.py`](config_tests/motor_test.py) — enables both
  motors and commands a small relative move on each. Verifies that the
  USB connections, encoder counts, and gear ratios in `config.yaml` are
  correctly wired up.
- [`config_tests/hardstop_test.py`](config_tests/hardstop_test.py) — runs
  current-threshold homing on each axis in sequence, polling position +
  motor current + statusword at 10 Hz and logging each tick. The tuning
  knobs at the top of the file (`HOMING_DIRECTION`, `CURRENT_THRESHOLD_MA`,
  `HOMING_SPEED_RPM`) are useful for sizing the production homing
  parameters in `config.yaml`.

Tests in `config_tests/` bypass `MotionController`, so they are **not**
subject to the Python-side bounds and overcurrent checks. They drive
`EposAxis` directly. Use them only during commissioning.

## Safety limits

Three independent safety mechanisms protect the mechanism during motion:

1. **Workspace bounds** (`machine.bounds` in `config.yaml`) — `move_to`
   rejects any (x, y) outside the rectangle before issuing a command.
2. **Post-homing position limits** (`axes.*.joint_travel_deg`) — after a
   successful `home_all`, the firmware writes EPOS object `0x607D`
   (Software Position Limit) on each axis to `[-travel, 0]` for positive
   homing or `[0, +travel]` for negative homing. Any move past the home
   point or beyond the configured travel is refused by the EPOS
   firmware itself (statusword bit 11 "Internal limit active"). The
   Python-side `move_to` and `rotate_axis` also pre-check against these
   limits and raise `BoundsError` before issuing the hardware command.
3. **Overcurrent ceiling** (`machine.overcurrent_limit_mA`) — during a
   move, `MotionController` polls `VCS_GetCurrentIsAveraged` on both
   axes at ~50 Hz. If either exceeds the limit, both axes are halted
   and `OvercurrentError` is raised. Homing is exempt (it intentionally
   develops up to `home_current_threshold_mA`); the overcurrent limit
   should be set above the homing threshold as a safety net.

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
`MotionController`: `home_all`, `move_to`, `jog`, `rotate_axis`, `halt`, `stop`,
`set_feed_mm_s`. Owns one EPOS driver *per* axis (each EPOS2 has its own USB
cable), runs a 20 Hz state-poll thread, broadcasts via subscriber callbacks.
Converts joint deg ↔ motor encoder counts using each axis's gear ratio.
After homing, writes per-axis software position limits to EPOS object `0x607D`
and caches them for Python-side pre-checks; `move_to` waits via an overcurrent
guard that polls both axes' current at ~50 Hz.

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
  core/             kinematics, motion, probe, scan, limits
  drivers/          EPOS2 ctypes wrapper + simulator
  web/              Flask-SocketIO server + static frontend
run.py              web-UI entrypoint
cli.py              command-line front-end (home, move, rotate, status, stop)
config.yaml         machine / kinematics / axes / probe / network config
tests/              pytest suite (kinematics round-trip, limits, scan sim)
config_tests/       hardware bring-up scripts (motor + hardstop homing)
EPOS_Linux_Library/ vendored Maxon SDK + installer (libEposCmd, libftd2xx)
```
