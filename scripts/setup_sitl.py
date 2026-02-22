#!/usr/bin/env python3
"""
Setup ArduPilot SITL for aerpawlib development.

Clones ArduPilot,
initializes submodules, and compiles SITL for Copter and Rover.

Matches the Dockerfile SITL setup. Run manually with:
    aerpawlib-setup-sitl

Or it runs automatically when you: pip install -e .[dev]
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


ARDUPILOT_REPO = "https://github.com/ArduPilot/ardupilot.git"
ARDUPILOT_BRANCH = os.getenv("ARDUPILOT_BRANCH", "Copter-4.6.3")


def _project_root() -> Path:
    """Project root (aerpawlib-vehicle-control)."""
    return Path(__file__).resolve().parent.parent


def _workarea() -> Path:
    """Where to clone ardupilot. Prefer project_root/ardupilot for conftest compatibility."""
    root = _project_root()
    return root / "ardupilot"


def _run(cmd: list[str], cwd: Path | None = None, env: dict | None = None) -> bool:
    """Run command; return True on success."""
    merged = os.environ.copy()
    if env:
        merged.update(env)
    try:
        subprocess.run(cmd, cwd=cwd or _project_root(), env=merged, check=True)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Command failed (exit {e.returncode}): {' '.join(cmd)}", file=sys.stderr)
        return False
    except FileNotFoundError:
        print(f"Command not found: {cmd[0]}", file=sys.stderr)
        return False


def setup_sitl(skip_compile: bool = False) -> bool:
    """
    Clone ArduPilot, init submodules, optionally compile SITL.

    Returns True if successful.
    """
    root = _project_root()
    workarea = root / "ardupilot"

    if workarea.exists():
        print(f"ArduPilot already exists at {workarea}")
        if skip_compile:
            return True
    else:
        print(f"Cloning ArduPilot ({ARDUPILOT_BRANCH}) to {workarea}...")
        if not _run(
            ["git", "clone", ARDUPILOT_REPO, "--branch", ARDUPILOT_BRANCH, "--single-branch", str(workarea)]
        ):
            return False

        print("Initializing submodules...")
        if not _run(["git", "submodule", "update", "--recursive", "--init"], cwd=workarea):
            return False

    if skip_compile:
        return True

    autotest = workarea / "Tools" / "autotest"
    if not autotest.exists():
        print(f"autotest dir not found: {autotest}", file=sys.stderr)
        return False

    env = os.environ.copy()
    env["ARDUPILOT_HOME"] = str(workarea)
    env["PATH"] = f"{workarea}:{env.get('PATH', '')}"
    # Prevent sim_vehicle from opening a new terminal window
    env["DISPLAY"] = env.get("DISPLAY", "")

    print("Compiling SITL...")
    if not _run(
        ["./waf", "configure", "--board", "sitl"],
        cwd=workarea,
        env=env,
    ):
        return False

    print("SITL setup complete.")
    print(f"  ARDUPILOT_HOME={workarea}")
    print(f"  sim_vehicle.py at {autotest / 'sim_vehicle.py'}")
    return True


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Setup ArduPilot SITL for aerpawlib")
    parser.add_argument(
        "--skip-compile",
        action="store_true",
        help="Only clone/init, do not compile SITL",
    )
    args = parser.parse_args()

    ok = setup_sitl(skip_compile=args.skip_compile)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
