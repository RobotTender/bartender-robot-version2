#!/usr/bin/env python3
"""Compatibility wrapper for planner location.

실제 구현은 src/bartender_action/robot_action_planner.py 에 있다.
"""

import os
import sys


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from bartender_action.robot_action_planner import main, run_robot_action  # noqa: F401


if __name__ == "__main__":
    raise SystemExit(main())
