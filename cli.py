"""Command-line interface for the Wood FPM firmware.

Examples:
    python3 cli.py home                  # drive both axes into their hard stops
    python3 cli.py move 5 0              # move workspace to (x=5, y=0) mm
    python3 cli.py move 0 0              # workspace home
    python3 cli.py rotate theta -45      # rotate the theta joint by -45 degrees
    python3 cli.py rotate phi  -30       # rotate the phi   joint by -30 degrees
    python3 cli.py status                # print current joint angles + xyz
    python3 cli.py stop                  # halt motion and disable drives

Each invocation opens the EPOS connections, runs the command, and exits.
Homing state and EPOS-side software position limits persist on the
controllers across invocations (until power cycle), so a single `home` at
the start of a session is enough.
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
from pathlib import Path

import yaml

from src.core.kinematics import PPMKinematics
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


def cmd_home(motion: MotionController, args) -> int:
    print("homing both axes (this drives each joint into its hard stop)...")
    motion.home_all()
    print("homed")
    return 0


def cmd_move(motion: MotionController, args) -> int:
    print(f"moving to workspace ({args.x:+.3f}, {args.y:+.3f}) mm")
    motion.move_to(args.x, args.y)
    print("done")
    return 0


def cmd_rotate(motion: MotionController, args) -> int:
    print(f"rotating {args.axis} by {args.deg:+.3f}° (joint)")
    motion.rotate_axis(args.axis, args.deg)
    print("done")
    return 0


def cmd_status(motion: MotionController, args) -> int:
    theta_counts = motion.axis_theta.position()
    phi_counts = motion.axis_phi.position()
    theta_deg = motion.axis_theta_cfg.counts_to_deg(theta_counts)
    phi_deg = motion.axis_phi_cfg.counts_to_deg(phi_counts)
    x, y, z = motion.kin.forward_kinematics(
        math.radians(theta_deg), math.radians(phi_deg)
    )
    print(f"theta: {theta_deg:+7.2f}°  ({theta_counts:+7d} counts)")
    print(f"phi:   {phi_deg:+7.2f}°  ({phi_counts:+7d} counts)")
    print(f"xyz:   ({x:+.3f}, {y:+.3f}, {z:+.3f}) mm (relative to home)")
    return 0


def cmd_stop(motion: MotionController, args) -> int:
    motion.stop()
    print("halted and disabled")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Wood FPM firmware CLI")
    parser.add_argument("--config", default=Path("config.yaml"), type=Path)
    parser.add_argument("--debug", action="store_true")
    sub = parser.add_subparsers(dest="command", required=True)

    p_home = sub.add_parser("home", help="drive both axes into hard stops")
    p_home.set_defaults(func=cmd_home)

    p_move = sub.add_parser("move", help="move to workspace (x, y) in mm")
    p_move.add_argument("x", type=float)
    p_move.add_argument("y", type=float)
    p_move.set_defaults(func=cmd_move)

    p_rot = sub.add_parser("rotate", help="rotate one joint by N degrees (signed)")
    p_rot.add_argument("axis", choices=["theta", "phi"])
    p_rot.add_argument("deg", type=float)
    p_rot.set_defaults(func=cmd_rotate)

    p_st = sub.add_parser("status", help="print current joint angles + xyz")
    p_st.set_defaults(func=cmd_status)

    p_stop = sub.add_parser("stop", help="halt motion and disable drives")
    p_stop.set_defaults(func=cmd_stop)

    return parser


def main() -> int:
    args = build_parser().parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)-5s %(name)s: %(message)s",
    )

    cfg = yaml.safe_load(args.config.read_text())
    motion = build_motion(cfg)

    try:
        return args.func(motion, args)
    except (BoundsError, OvercurrentError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    except EposError as e:
        print(f"EPOS ERROR: {e}", file=sys.stderr)
        return 1
    finally:
        motion.shutdown()


if __name__ == "__main__":
    sys.exit(main())
