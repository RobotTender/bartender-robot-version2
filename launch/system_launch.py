#!/usr/bin/env python3
import csv
import os
import signal
import socket
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _parse_cli_overrides(argv):
    overrides = {}
    for token in argv or []:
        if ':=' not in token:
            continue
        key, value = token.split(':=', 1)
        key = key.strip()
        if not key:
            continue
        if key == 'run_ui':
            key = 'run_frontend'
        if key == 'rt_host':
            key = 'robot_rt_host'
        if key in ('run_web', 'run_webui'):
            key = 'run_user_frontend'
        overrides[key] = value
    run_robot = str(overrides.get('run_robot', 'true')).strip().lower()
    if run_robot != 'true' and 'robot_mode' not in overrides:
        overrides['robot_mode'] = 'virtual'
    mode = str(overrides.get('robot_mode', '')).strip().lower()
    if mode == 'virtual' and 'robot_host' not in overrides:
        overrides['robot_host'] = '127.0.0.1'
    if mode == 'real' and 'robot_gz' not in overrides:
        overrides['robot_gz'] = 'false'
    if mode == 'virtual' and 'robot_gz' not in overrides:
        overrides['robot_gz'] = 'true'
    if mode == 'real' and 'robot_rt_host' not in overrides:
        detected_rt_host = _detect_local_ip_for_target(overrides.get('robot_host', '110.120.1.68'))
        if detected_rt_host:
            overrides['robot_rt_host'] = detected_rt_host
    return overrides


def _is_true_text(value, default=False):
    text = str(value if value is not None else '').strip().lower()
    if text in ('1', 'true', 'yes', 'on'):
        return True
    if text in ('0', 'false', 'no', 'off'):
        return False
    return bool(default)


def _detect_local_ip_for_target(target_host):
    host = str(target_host or '').strip()
    if not host:
        return None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect((host, 1))
            return str(sock.getsockname()[0] or '').strip() or None
        finally:
            sock.close()
    except Exception:
        return None


def _load_saved_runtime_toggle_defaults(project_root):
    param_file = os.path.join(str(project_root), 'config', 'parameter.csv')
    defaults = (True, True, True)  # vision1, vision2, robot
    if not os.path.isfile(param_file):
        return defaults
    rows = {}
    try:
        with open(param_file, 'r', encoding='utf-8', newline='') as f:
            for row in csv.reader(f):
                if not row:
                    continue
                key = str(row[0]).strip()
                if not key or key.lower() == 'name':
                    continue
                rows[key] = [str(v).strip() for v in row[1:] if str(v).strip()]
    except Exception:
        return defaults

    def _as_bool_from_row(key, default):
        vals = rows.get(str(key), [])
        raw = vals[0] if vals else ("1" if default else "0")
        text = str(raw).strip().lower()
        if text in ('1', 'true', 'yes', 'on'):
            return True
        if text in ('0', 'false', 'no', 'off'):
            return False
        return bool(default)

    vision1_on = _as_bool_from_row('top_status_vision_enabled', True)
    vision2_on = _as_bool_from_row('top_status_vision2_enabled', True)
    robot_on = _as_bool_from_row('top_status_robot_enabled', True)
    return bool(vision1_on), bool(vision2_on), bool(robot_on)


def _collect_processes_by_pattern(pattern):
    try:
        output = subprocess.check_output(['pgrep', '-af', pattern], text=True)
    except Exception:
        return {}
    found = {}
    for raw in str(output).splitlines():
        line = str(raw).strip()
        if not line:
            continue
        parts = line.split(None, 1)
        if not parts:
            continue
        try:
            pid = int(parts[0])
        except Exception:
            continue
        if pid == os.getpid():
            continue
        found[pid] = parts[1] if len(parts) > 1 else ''
    return found


GAZEBO_PROCESS_PATTERNS = (
    r'ruby .*/gz sim',
    r'ruby .*/gz gui',
    r'(^| )gz sim( |$)',
    r'(^| )gz gui( |$)',
    r'(^| )ign gazebo( |$)',
    r'ign-gazebo',
    r'gz-sim',
    r'gz-gui',
    r'gzserver',
    r'gzclient',
    r'gazebo( |$)',
)


def _collect_gazebo_processes():
    found = {}
    for pattern in GAZEBO_PROCESS_PATTERNS:
        found.update(_collect_processes_by_pattern(pattern))
    return found


def _wait_for_exit(pids, timeout_sec):
    remaining = set(int(pid) for pid in pids)
    deadline = time.monotonic() + max(0.0, float(timeout_sec))
    while remaining and time.monotonic() < deadline:
        for pid in list(remaining):
            try:
                os.kill(pid, 0)
            except OSError:
                remaining.discard(pid)
        if remaining:
            time.sleep(0.05)
    return remaining


def _cleanup_lingering_gazebo_processes(enabled=True, baseline_pids=None):
    if not enabled:
        return
    found = _collect_gazebo_processes()
    if baseline_pids:
        keep = {int(pid) for pid in baseline_pids}
        found = {pid: cmd for pid, cmd in found.items() if int(pid) not in keep}
    if not found:
        return
    remaining = set(found.keys())
    for sig, wait_sec in (
        (signal.SIGINT, 1.0),
        (signal.SIGTERM, 1.0),
        (signal.SIGKILL, 0.2),
    ):
        if not remaining:
            break
        for pid in list(remaining):
            try:
                os.kill(pid, sig)
            except OSError:
                remaining.discard(pid)
        remaining = _wait_for_exit(remaining, wait_sec)
    cleaned = sorted(set(found.keys()) - set(remaining))
    if cleaned:
        print(f"[system_launch] Gazebo 잔여 프로세스 정리: {', '.join(str(pid) for pid in cleaned)}")
    if remaining:
        print(f"[system_launch] Gazebo 잔여 프로세스 정리 실패: {', '.join(str(pid) for pid in sorted(remaining))}")


def generate_launch_description(overrides=None):
    this_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(this_dir, '..'))
    doosan_share = get_package_share_directory('dsr_bringup2')
    vision1_on, vision2_on, robot_on = _load_saved_runtime_toggle_defaults(project_root)
    run_robot_default = 'true' if robot_on else 'false'
    run_sensors_default = 'true' if (vision1_on or vision2_on) else 'false'
    print(
        f"[system_launch] 저장된 토글 기반 기본값: "
        f"robot={robot_on} -> run_robot={run_robot_default}, "
        f"vision1={vision1_on}, vision2={vision2_on} -> run_sensors={run_sensors_default}"
    )

    args = [
        DeclareLaunchArgument('run_robot', default_value=run_robot_default),
        DeclareLaunchArgument('robot_mode', default_value='real'),
        DeclareLaunchArgument('robot_host', default_value='110.120.1.68'),
        DeclareLaunchArgument('robot_rt_host', default_value='192.168.137.50'),
        DeclareLaunchArgument('robot_model', default_value='e0509'),
        DeclareLaunchArgument('robot_gz', default_value='false'),
        DeclareLaunchArgument('run_sensors', default_value=run_sensors_default),
        DeclareLaunchArgument('run_camera1', default_value='true' if vision1_on else 'false'),
        DeclareLaunchArgument('run_camera2', default_value='true' if vision2_on else 'false'),
        DeclareLaunchArgument('run_frontend', default_value='true'),
        # Canonical flag for end-user web frontend execution.
        DeclareLaunchArgument('run_user_frontend', default_value='true'),
        DeclareLaunchArgument('webui_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('webui_port', default_value='8000'),
        DeclareLaunchArgument('webui_order_start_enabled', default_value='true'),
        DeclareLaunchArgument('sequence_api_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('sequence_api_port', default_value='8765'),
        DeclareLaunchArgument('vision_python', default_value=os.environ.get('BARTENDER_VISION_PYTHON', '')),
    ]

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(doosan_share, 'launch', 'dsr_bringup2_gazebo.launch.py')
        ),
        launch_arguments={
            'mode': LaunchConfiguration('robot_mode'),
            'host': LaunchConfiguration('robot_host'),
            'rt_host': LaunchConfiguration('robot_rt_host'),
            'model': LaunchConfiguration('robot_model'),
            'gz': LaunchConfiguration('robot_gz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('run_robot')),
    )

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'realsense_launch.py')),
        launch_arguments={
            'run_camera1': LaunchConfiguration('run_camera1'),
            'run_camera2': LaunchConfiguration('run_camera2'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('run_sensors')),
    )

    frontend_proc = ExecuteProcess(
        cmd=[sys.executable, os.path.join(project_root, 'src', 'frontend', 'developer_frontend.py')],
        output='screen',
        emulate_tty=True,
        additional_env={
            'BARTENDER_ROBOT_MODE_HINT': LaunchConfiguration('robot_mode'),
            'BARTENDER_ROBOT_MODEL_HINT': LaunchConfiguration('robot_model'),
            'BARTENDER_ROBOT_HOST_HINT': LaunchConfiguration('robot_host'),
            'USER_FRONTEND_HOST': LaunchConfiguration('webui_host'),
            'USER_FRONTEND_PORT': LaunchConfiguration('webui_port'),
            'USER_FRONTEND_ORDER_START_ENABLED': LaunchConfiguration('webui_order_start_enabled'),
            'VOICE_ORDER_WEBUI_HOST': LaunchConfiguration('webui_host'),
            'VOICE_ORDER_WEBUI_PORT': LaunchConfiguration('webui_port'),
            'VOICE_ORDER_WEBUI_ORDER_START_ENABLED': LaunchConfiguration('webui_order_start_enabled'),
            'BARTENDER_SEQUENCE_API_HOST': LaunchConfiguration('sequence_api_host'),
            'BARTENDER_SEQUENCE_API_PORT': LaunchConfiguration('sequence_api_port'),
            'BARTENDER_VISION_PYTHON': LaunchConfiguration('vision_python'),
        },
        condition=IfCondition(LaunchConfiguration('run_frontend')),
    )

    user_frontend_proc = ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.join(project_root, 'src', 'frontend', 'user_frontend.py'),
            '--host',
            LaunchConfiguration('webui_host'),
            '--port',
            LaunchConfiguration('webui_port'),
            '--order-start-enabled',
            LaunchConfiguration('webui_order_start_enabled'),
        ],
        output='screen',
        emulate_tty=True,
        additional_env={
            'USER_FRONTEND_HOST': LaunchConfiguration('webui_host'),
            'USER_FRONTEND_PORT': LaunchConfiguration('webui_port'),
            'USER_FRONTEND_ORDER_START_ENABLED': LaunchConfiguration('webui_order_start_enabled'),
            'VOICE_ORDER_WEBUI_HOST': LaunchConfiguration('webui_host'),
            'VOICE_ORDER_WEBUI_PORT': LaunchConfiguration('webui_port'),
            'VOICE_ORDER_WEBUI_ORDER_START_ENABLED': LaunchConfiguration('webui_order_start_enabled'),
            'BARTENDER_SEQUENCE_API_HOST': LaunchConfiguration('sequence_api_host'),
            'BARTENDER_SEQUENCE_API_PORT': LaunchConfiguration('sequence_api_port'),
        },
        condition=IfCondition(LaunchConfiguration('run_user_frontend')),
    )

    shutdown_on_frontend_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=frontend_proc,
            on_exit=[EmitEvent(event=Shutdown(reason='developer_frontend exited'))],
        )
    )

    override_actions = [SetLaunchConfiguration(name, value) for name, value in (overrides or {}).items()]

    return LaunchDescription(
        args
        + override_actions
        + [robot_launch, sensor_launch, user_frontend_proc, frontend_proc, shutdown_on_frontend_exit]
    )


def main(argv=None):
    argv = argv or []
    overrides = _parse_cli_overrides(argv)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(generate_launch_description(overrides))
    cleanup_gazebo = bool(
        _is_true_text((overrides or {}).get('run_robot', 'true'), default=True)
        and _is_true_text((overrides or {}).get('robot_gz', 'true'), default=True)
    )
    baseline_gazebo_pids = set()
    if cleanup_gazebo:
        baseline_gazebo_pids = set(_collect_gazebo_processes().keys())
    try:
        return ls.run()
    finally:
        _cleanup_lingering_gazebo_processes(
            enabled=cleanup_gazebo,
            baseline_pids=baseline_gazebo_pids,
        )


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
