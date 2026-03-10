#!/usr/bin/env python3
import csv
import os
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _normalize_serial_text(value):
    s = str(value or "").strip()
    if not s:
        return ""
    if s.startswith("_"):
        s = s[1:]
    return s


def _launch_serial_text(value):
    s = _normalize_serial_text(value)
    return f"_{s}" if s else ""


def _load_camera_serials(project_root):
    default_1 = '313522301601'
    default_2 = '311322302867'
    param_file = os.path.join(project_root, 'config', 'parameter.csv')
    if not os.path.isfile(param_file):
        return default_1, default_2
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
        return default_1, default_2
    serial_1 = _normalize_serial_text((rows.get('vision1_serial') or [''])[0] if rows.get('vision1_serial') else '')
    serial_2 = _normalize_serial_text((rows.get('vision2_serial') or [''])[0] if rows.get('vision2_serial') else '')
    return serial_1 or default_1, serial_2 or default_2


def _parse_cli_overrides(argv):
    overrides = {}
    for token in argv or []:
        if ':=' not in token:
            continue
        key, value = token.split(':=', 1)
        key = key.strip()
        if not key:
            continue
        overrides[key] = value
    return overrides


def generate_launch_description(overrides=None):
    project_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    serial_1, serial_2 = _load_camera_serials(project_root)

    args = [
        DeclareLaunchArgument('enable_depth', default_value='true'),
        DeclareLaunchArgument('enable_color', default_value='true'),
        DeclareLaunchArgument('enable_sync', default_value='true'),
        DeclareLaunchArgument('align_depth_enable', default_value='true'),
        DeclareLaunchArgument('run_camera1', default_value='true'),
        DeclareLaunchArgument('run_camera2', default_value='true'),
        DeclareLaunchArgument('camera1_namespace', default_value='camera'),
        DeclareLaunchArgument('camera1_name', default_value='camera'),
        DeclareLaunchArgument('camera1_serial_no', default_value=_launch_serial_text(serial_1) or '_313522301601'),
        DeclareLaunchArgument('camera2_namespace', default_value='camera2'),
        DeclareLaunchArgument('camera2_name', default_value='camera'),
        DeclareLaunchArgument('camera2_serial_no', default_value=_launch_serial_text(serial_2) or '_311322302867'),
    ]

    camera1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('run_camera1')),
        launch_arguments={
            'enable_depth': LaunchConfiguration('enable_depth'),
            'enable_color': LaunchConfiguration('enable_color'),
            'enable_sync': LaunchConfiguration('enable_sync'),
            'align_depth.enable': LaunchConfiguration('align_depth_enable'),
            'camera_namespace': LaunchConfiguration('camera1_namespace'),
            'camera_name': LaunchConfiguration('camera1_name'),
            'serial_no': LaunchConfiguration('camera1_serial_no'),
        }.items(),
    )

    camera2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('run_camera2')),
        launch_arguments={
            'enable_depth': LaunchConfiguration('enable_depth'),
            'enable_color': LaunchConfiguration('enable_color'),
            'enable_sync': LaunchConfiguration('enable_sync'),
            'align_depth.enable': LaunchConfiguration('align_depth_enable'),
            'camera_namespace': LaunchConfiguration('camera2_namespace'),
            'camera_name': LaunchConfiguration('camera2_name'),
            'serial_no': LaunchConfiguration('camera2_serial_no'),
        }.items(),
    )

    override_actions = [SetLaunchConfiguration(name, value) for name, value in (overrides or {}).items()]
    return LaunchDescription(args + override_actions + [camera1, camera2])


def main(argv=None):
    argv = argv or []
    ls = LaunchService(argv=argv)
    ls.include_launch_description(generate_launch_description(_parse_cli_overrides(argv)))
    return ls.run()


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
