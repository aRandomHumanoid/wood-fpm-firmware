"""Interactive command-line interface for the Wood FPM firmware.

Run:
    python3 cli.py

Type `help` at the prompt to list commands, `help <cmd>` for details, or
`quit` (also `exit` / Ctrl-D) to leave. Arrow keys give command history.

EPOS connections are opened once on startup and closed on exit; commands
within a session reuse them. EPOS-side state (homing-attained flag and
the post-homing software position limits) persists on the controllers
until power-cycle, so re-launching the CLI doesn't require re-homing.
"""

from __future__ import annotations

import argparse
import cmd
import logging
import math
import sys
import time
from pathlib import Path

import yaml

try:
    import readline  # noqa: F401  -- enables arrow-key history at the prompt
except ImportError:
    pass

from src.core.kinematics import KinematicsError, PPMKinematics
from src.core.limits import (
    BoundsError,
    MachineBounds,
    MotionLimits,
    OvercurrentError,
)
from src.core.motion import AxisConfig, MotionController
from src.drivers.epos import EposError


def build_motion(cfg: dict) -> MotionController:
    m = cfg["machine"]
    bounds = MachineBounds(**m["bounds"])
    limits = MotionLimits(
        v_max_mm_s=m["v_max_mm_s"],
        a_max_mm_s2=m["a_max_mm_s2"],
        overcurrent_mA=m.get("overcurrent_limit_mA", 0),
    )
    k = cfg["kinematics"]
    kin = PPMKinematics(
        Lc=k["Lc"], H=k["H"], D=k["D"], G_deg=k["G"],
        theta_max_deg=k["theta_max_deg"],
    )
    axes = cfg["axes"]
    ax_theta = AxisConfig(name="theta", **axes["theta"])
    ax_phi = AxisConfig(name="phi", **axes["phi"])
    return MotionController(
        kin=kin, bounds=bounds, limits=limits,
        axis_theta=ax_theta, axis_phi=ax_phi,
        simulate=cfg["driver"]["simulate"],
        epos_library=cfg["driver"]["epos_library"],
        rapid_feed_mm_s=m.get("rapid_feed_mm_s", 15.0),
        scan_feed_mm_s=m.get("scan_feed_mm_s", 2.0),
    )


class FPMShell(cmd.Cmd):
    intro = (
        "Wood FPM interactive CLI. "
        "Type 'help' for commands, 'quit' to exit."
    )
    prompt = "fpm> "

    def __init__(self, motion: MotionController):
        super().__init__()
        self.motion = motion

    # ---- internal helper -------------------------------------------------

    def _run(self, fn, *args, label: str | None = "done"):
        """Run a motion op, catching expected errors so the shell stays up.
        Ctrl-C during a long-running op halts motion and returns to prompt."""
        try:
            fn(*args)
        except (BoundsError, OvercurrentError) as e:
            print(f"BLOCKED: {e}")
        except EposError as e:
            print(f"EPOS ERROR: {e}")
        except KeyboardInterrupt:
            print("\n^C — halting motion")
            try:
                self.motion.halt()
            except Exception:
                pass
        except Exception as e:
            print(f"ERROR: {e!r}")
        else:
            if label:
                print(label)

    # ---- commands --------------------------------------------------------

    def do_home(self, arg: str):
        """home — drive both joints into their hard stops, then set the
        post-homing software position limits."""
        if arg.strip():
            print("usage: home")
            return
        print("homing both axes (drives each into its hard stop)...")
        self._run(self.motion.home_all, label="homed")

    def do_move(self, arg: str):
        """move [abs|rel] X Y — workspace move in mm. Default is absolute.
            move 5 0          # absolute (x=5, y=0)
            move abs 5 0      # same as above
            move rel 1 0      # +1 mm in x from current position
        """
        parts = arg.split()
        mode = "abs"
        if parts and parts[0] in ("abs", "rel"):
            mode = parts.pop(0)
        if len(parts) != 2:
            print("usage: move [abs|rel] X Y")
            return
        try:
            x, y = float(parts[0]), float(parts[1])
        except ValueError:
            print("X and Y must be numbers")
            return
        if mode == "abs":
            print(f"moving to ({x:+.3f}, {y:+.3f}) mm")
            self._run(self.motion.move_to, x, y)
        else:
            print(f"jogging by ({x:+.3f}, {y:+.3f}) mm")
            self._run(self.motion.jog, x, y)

    def do_rotate(self, arg: str):
        """rotate [abs|rel] {theta|phi} DEG — rotate one joint by/to N°.
        Default is relative.
            rotate theta -45      # relative: -45° from current pose
            rotate rel theta -45  # same as above
            rotate abs theta -45  # absolute: go to joint angle -45°
        """
        parts = arg.split()
        mode = "rel"
        if parts and parts[0] in ("abs", "rel"):
            mode = parts.pop(0)
        if len(parts) != 2:
            print("usage: rotate [abs|rel] {theta|phi} DEG")
            return
        name, deg_s = parts
        if name not in ("theta", "phi"):
            print("axis must be 'theta' or 'phi'")
            return
        try:
            deg = float(deg_s)
        except ValueError:
            print("DEG must be a number")
            return
        if mode == "abs":
            print(f"rotating {name} to {deg:+.3f}°")
        else:
            print(f"rotating {name} by {deg:+.3f}°")
        self._run(self.motion.rotate_axis, name, deg, mode == "abs")

    def do_status(self, arg: str):
        """status — print current joint angles, motor counts, and xyz."""
        if arg.strip():
            print("usage: status")
            return
        try:
            theta_counts = self.motion.axis_theta.position()
            phi_counts = self.motion.axis_phi.position()
        except EposError as e:
            print(f"EPOS ERROR: {e}")
            return
        theta_deg = self.motion.axis_theta_cfg.counts_to_deg(theta_counts)
        phi_deg = self.motion.axis_phi_cfg.counts_to_deg(phi_counts)
        theta_kdeg = self.motion.axis_theta_cfg.counts_to_kinematics_deg(theta_counts)
        phi_kdeg = self.motion.axis_phi_cfg.counts_to_kinematics_deg(phi_counts)
        print(f"theta: {theta_deg:+7.2f}°  ({theta_counts:+7d} counts)  [kin {theta_kdeg:+7.2f}°]")
        print(f"phi:   {phi_deg:+7.2f}°  ({phi_counts:+7d} counts)  [kin {phi_kdeg:+7.2f}°]")
        # Forward kinematics is only well-defined inside the workspace
        # envelope. Outside it (unhomed, rotated past the envelope), the
        # PPM trilateration goes singular.
        try:
            x, y, z = self.motion.kin.forward_kinematics(
                math.radians(theta_kdeg), math.radians(phi_kdeg)
            )
        except KinematicsError as e:
            print(f"xyz:   <unreachable> ({e})")
            return
        print(f"xyz:   ({x:+.3f}, {y:+.3f}, {z:+.3f}) mm (relative to home)")

    WIGGLE_DEFAULT_DEG = 5.0
    WIGGLE_VELOCITY_RPM = 20

    def do_wiggle(self, arg: str):
        """wiggle [DEG] — rotate BOTH joints DEG° CW then back to start.
        Slow (20 motor-RPM) by design — useful for verifying both motors
        respond, the bounds check works, and motion is smooth.
            wiggle           # ±5°  on both axes (default)
            wiggle 3         # ±3°  on both axes
        """
        parts = arg.split()
        deg = self.WIGGLE_DEFAULT_DEG
        if parts:
            try:
                deg = float(parts[0])
            except ValueError:
                print("usage: wiggle [DEG]")
                return
        print(f"wiggling both joints +{deg:.1f}° then back at {self.WIGGLE_VELOCITY_RPM} rpm")

        try:
            self.motion.rotate_both_axes(deg, velocity_rpm=self.WIGGLE_VELOCITY_RPM)
            print(f"forward (+{deg:.1f}°)")
            time.sleep(0.3)
            self.motion.rotate_both_axes(-deg, velocity_rpm=self.WIGGLE_VELOCITY_RPM)
            print(f"back   (-{deg:.1f}°)  → at start")
        except (BoundsError, OvercurrentError) as e:
            print(f"BLOCKED: {e}")
        except EposError as e:
            print(f"EPOS ERROR: {e}")
        except KeyboardInterrupt:
            print("\n^C — halting motion")
            try:
                self.motion.halt()
            except Exception:
                pass

    def do_stop(self, arg: str):
        """stop — E-stop: halt motion and disable both drives.
        Use 'enable' to re-engage afterward."""
        self.motion.stop()
        print("halted and disabled — run 'enable' or 'home' to re-engage")

    def do_clear_faults(self, arg: str):
        """clear_faults — clear fault state on both drives.
        Run this if `status` / a previous command left an axis in a fault
        and you want to recover without re-homing. Follow with `enable`."""
        if arg.strip():
            print("usage: clear_faults")
            return
        for axis, name in (
            (self.motion.axis_theta, "theta"),
            (self.motion.axis_phi,   "phi"),
        ):
            was_faulted = False
            if hasattr(axis, "fault_state"):
                try:
                    was_faulted = axis.fault_state()
                except EposError:
                    pass
            try:
                axis.clear_fault()
                print(f"{name}: cleared" + ("  (was faulted)" if was_faulted else ""))
            except EposError as e:
                print(f"{name}: EPOS ERROR {e}")

    def do_enable(self, arg: str):
        """enable — re-engage both drives in profile-position mode
        (use after 'stop' or a fault)."""
        try:
            for axis in (self.motion.axis_theta, self.motion.axis_phi):
                axis.clear_fault()
                axis.enable()
                axis.activate_pp_mode()
        except EposError as e:
            print(f"EPOS ERROR: {e}")
            return
        print("enabled (profile-position mode)")

    def do_quit(self, arg: str):
        """quit — exit the shell."""
        return True

    do_exit = do_quit

    def do_EOF(self, arg: str):
        """Ctrl-D — exit the shell."""
        print()
        return True

    # ---- shell behavior --------------------------------------------------

    def emptyline(self):
        pass  # blank line is a no-op (default would repeat last command)

    def default(self, line: str):
        tok = line.split()[0] if line.split() else ""
        print(f"unknown command: {tok!r} (type 'help' for the list)")


def main() -> int:
    parser = argparse.ArgumentParser(description="Wood FPM interactive CLI")
    parser.add_argument("--config", default=Path("config.yaml"), type=Path)
    parser.add_argument("--debug", action="store_true",
                        help="verbose driver logging at the prompt")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.WARNING,
        format="%(asctime)s %(levelname)-5s %(name)s: %(message)s",
    )

    cfg = yaml.safe_load(args.config.read_text())
    motion = build_motion(cfg)

    try:
        FPMShell(motion).cmdloop()
    except KeyboardInterrupt:
        print("\n^C — exiting")
    finally:
        motion.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
