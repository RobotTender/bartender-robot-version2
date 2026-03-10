#!/usr/bin/env python3
import os
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetLaunchConfiguration
from launch.conditions import IfCondition
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
        overrides[key] = value
    return overrides


def generate_launch_description(overrides=None):
    project_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

    args = [
        DeclareLaunchArgument('run_vision1_calibration', default_value='false'),
        DeclareLaunchArgument('run_vision2_calibration', default_value='false'),
        DeclareLaunchArgument('pattern_cols', default_value='7'),
        DeclareLaunchArgument('pattern_rows', default_value='9'),
    ]

    vision1 = ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.join(project_root, 'src', 'vision', 'camera_eye_to_hand_robot_calibration.py'),
            '--image-topic', '/camera/camera/color/image_raw',
            '--depth-topic', '/camera/camera/aligned_depth_to_color/image_raw',
            '--camera-info-topic', '/camera/camera/color/camera_info',
            '--output-meta-topic', '/vision1/calibration/meta',
            '--cols', LaunchConfiguration('pattern_cols'),
            '--rows', LaunchConfiguration('pattern_rows'),
            '--panel', '1',
        ],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('run_vision1_calibration')),
    )

    vision2 = ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.join(project_root, 'src', 'vision', 'camera_eye_to_hand_robot_calibration.py'),
            '--image-topic', '/camera2/camera/color/image_raw',
            '--depth-topic', '/camera2/camera/aligned_depth_to_color/image_raw',
            '--camera-info-topic', '/camera2/camera/color/camera_info',
            '--output-meta-topic', '/vision2/calibration/meta',
            '--cols', LaunchConfiguration('pattern_cols'),
            '--rows', LaunchConfiguration('pattern_rows'),
            '--panel', '2',
        ],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('run_vision2_calibration')),
    )

    override_actions = [SetLaunchConfiguration(name, value) for name, value in (overrides or {}).items()]
    return LaunchDescription(args + override_actions + [vision1, vision2])


def main(argv=None):
    argv = argv or []
    ls = LaunchService(argv=argv)
    ls.include_launch_description(generate_launch_description(_parse_cli_overrides(argv)))
    return ls.run()


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
