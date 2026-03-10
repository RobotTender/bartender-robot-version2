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
        DeclareLaunchArgument('run_drink_detection', default_value='false'),
        DeclareLaunchArgument('run_glass_fill_level', default_value='false'),
    ]

    drink_detection = ExecuteProcess(
        cmd=[sys.executable, os.path.join(project_root, 'src', 'vision', 'drink_detection.py')],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('run_drink_detection')),
    )

    glass_fill_level = ExecuteProcess(
        cmd=[sys.executable, os.path.join(project_root, 'src', 'vision', 'glass_fill_level.py')],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('run_glass_fill_level')),
    )

    override_actions = [SetLaunchConfiguration(name, value) for name, value in (overrides or {}).items()]
    return LaunchDescription(args + override_actions + [drink_detection, glass_fill_level])


def main(argv=None):
    argv = argv or []
    ls = LaunchService(argv=argv)
    ls.include_launch_description(generate_launch_description(_parse_cli_overrides(argv)))
    return ls.run()


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
