"""Interactive command-line interface for the Wood FPM firmware.

Run:
    python3 cli.py

Type `help` at the prompt to list commands, `help <cmd>` for details, or
`quit` (also `exit` / Ctrl-D) to leave. Arrow keys give command history.

The CLI shares the same MotionController + serial Marlin connection as the
web UI. State (homed flag, current position) lives on the Marlin board and
persists across CLI sessions until Marlin power-cycles.
"""

from __future__ import annotations

import argparse
import cmd
import logging
import sys
from pathlib import Path

import yaml

try:
    import readline  # noqa: F401  -- enables arrow-key history at the prompt
except ImportError:
    pass

from src.core.limits import BoundsError, MachineBounds, MotionLimits
from src.core.motion import MotionController
from src.drivers.marlin import MarlinError


def build_motion(cfg: dict) -> MotionController:
    m = cfg["machine"]
    bounds = MachineBounds(**m["bounds"])
    limits = MotionLimits(v_max_mm_s=m["v_max_mm_s"], a_max_mm_s2=m["a_max_mm_s2"])
    marlin_cfg = cfg.get("marlin", {})
    return MotionController(
        bounds=bounds, limits=limits,
        marlin_port=marlin_cfg.get("port", "/dev/ttyUSB0"),
        marlin_baudrate=marlin_cfg.get("baudrate", 115200),
        x_axis=marlin_cfg.get("x_axis", "X"),
        y_axis=marlin_cfg.get("y_axis", "Y"),
        z_axis=marlin_cfg.get("z_axis", "Z"),
        simulate=cfg["driver"]["simulate"],
        rapid_feed_mm_s=m.get("rapid_feed_mm_s", 15.0),
        scan_feed_mm_s=m.get("scan_feed_mm_s", 2.0),
    )


class FPMShell(cmd.Cmd):
    intro = "Wood FPM interactive CLI. Type 'help' for commands, 'quit' to exit."
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
        except BoundsError as e:
            print(f"BLOCKED: {e}")
        except MarlinError as e:
            print(f"MARLIN ERROR: {e}")
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
        """home — run Marlin's G28 on both configured axes."""
        if arg.strip():
            print("usage: home")
            return
        print("homing both axes...")
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

    def do_status(self, arg: str):
        """status — print current workspace position from Marlin."""
        if arg.strip():
            print("usage: status")
            return
        try:
            pos = self.motion.marlin.position()
        except MarlinError as e:
            print(f"MARLIN ERROR: {e}")
            return
        x = pos.logical.get(self.motion._x_letter, 0.0)
        y = pos.logical.get(self.motion._y_letter, 0.0)
        z = pos.logical.get(self.motion._z_letter, 0.0)
        print(f"workspace: ({x:+.3f}, {y:+.3f}, {z:+.3f}) mm")

    def do_stop(self, arg: str):
        """stop — E-stop: halt motion and disable steppers (M410 + M84).
        Issue any move (or 'home') to re-enable the steppers automatically."""
        self.motion.stop()
        print("halted and disabled — issue a move or 'home' to re-engage")

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
