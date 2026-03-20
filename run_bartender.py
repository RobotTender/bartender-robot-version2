#!/usr/bin/env python3
import importlib.util
import os
import shlex
import subprocess
import sys
import ctypes
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent
SYSTEM_LAUNCH_PATH = REPO_ROOT / "launch" / "system_launch.py"
BOOTSTRAP_ENV_KEY = "BARTENDER_ENV_BOOTSTRAPPED"
DEFAULT_VISION_PYTHON_CANDIDATES = [
    REPO_ROOT / ".venv-vision" / "bin" / "python",
    REPO_ROOT / "venv-vision" / "bin" / "python",
    REPO_ROOT.parent / "venv-vision" / "bin" / "python",
]


def _find_ros_setup():
    preferred = [
        Path("/opt/ros/jazzy/setup.bash"),
        Path("/opt/ros/humble/setup.bash"),
    ]
    for path in preferred:
        if path.is_file():
            return path
    ros_root = Path("/opt/ros")
    if ros_root.is_dir():
        for path in sorted(ros_root.glob("*/setup.bash"), reverse=True):
            if path.is_file():
                return path
    return None


def _find_workspace_setup():
    for parent in REPO_ROOT.parents:
        candidate = parent / "install" / "setup.bash"
        if candidate.is_file():
            return candidate
    return None


def _needs_ros_bootstrap():
    if (
        importlib.util.find_spec("ament_index_python") is None
        or importlib.util.find_spec("launch") is None
    ):
        return True
    # Some shells keep python packages importable while LD_LIBRARY_PATH misses ROS libs.
    # In that case rclpy import fails at runtime with librcl_action.so not found.
    try:
        ctypes.CDLL("librcl_action.so")
    except OSError:
        return True
    return False

"""  """
def _reexec_with_ros_env():
    if os.environ.get(BOOTSTRAP_ENV_KEY) == "1":
        return None
    if not _needs_ros_bootstrap():
        return None
    ros_setup = _find_ros_setup()
    if ros_setup is None:
        return None
    workspace_setup = _find_workspace_setup()
    cmd_steps = [f"source {shlex.quote(str(ros_setup))}"]
    if workspace_setup is not None:
        cmd_steps.append(f"source {shlex.quote(str(workspace_setup))}")
    cmd_steps.append(f"export {BOOTSTRAP_ENV_KEY}=1")
    args = " ".join(shlex.quote(arg) for arg in sys.argv[1:])
    cmd_steps.append(
        f"exec {shlex.quote(sys.executable)} {shlex.quote(str(Path(__file__).resolve()))} {args}".rstrip()
    )
    completed = subprocess.run(["bash", "-lc", " && ".join(cmd_steps)])
    return int(completed.returncode)


def _load_launch_main():
    spec = importlib.util.spec_from_file_location("bartender_system_launch", SYSTEM_LAUNCH_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load launch file: {SYSTEM_LAUNCH_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.main


def _ensure_default_vision_python():
    # Keep app/backend interpreter as-is, but pin vision helpers to dedicated venv by default.
    if str(os.environ.get("BARTENDER_VISION_PYTHON", "")).strip():
        return
    for candidate in DEFAULT_VISION_PYTHON_CANDIDATES:
        if candidate.is_file():
            os.environ["BARTENDER_VISION_PYTHON"] = str(candidate)
            return


if __name__ == "__main__":
    _ensure_default_vision_python()
    bootstrap_exit = _reexec_with_ros_env()
    if bootstrap_exit is not None:
        raise SystemExit(bootstrap_exit)
    main = _load_launch_main()
    raise SystemExit(main(sys.argv[1:]))
