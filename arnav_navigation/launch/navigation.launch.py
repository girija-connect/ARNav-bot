"""
Navigation: localization (map_server + AMCL) and the Nav2 stack (planner, controller, costmaps, behaviours).
ros2 launch arnav_navigation navigation.launch.py
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def already_running():
    try:
        out = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=8).stdout
    except Exception:
        return False
    return any(n in out for n in ('/bt_navigator', '/planner_server', '/amcl'))


def launch_setup(context):
    if already_running():
        return [LogInfo(msg='ERROR: Nav2 is already running (found /bt_navigator, /planner_server or /amcl). '
                            'Stop it first: ~/ARNav_ws/src/arnav_bringup/scripts/stop_all.sh')]
    nav = get_package_share_directory('arnav_navigation')
    loc = get_package_share_directory('arnav_localization')
    nav2 = get_package_share_directory('nav2_bringup')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(loc, 'launch', 'localization.launch.py')),
            launch_arguments={'map': LaunchConfiguration('map')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': os.path.join(nav, 'config', 'navigation_params.yaml'),
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_composition': 'False',
            }.items()),
    ]


def generate_launch_description():
    maps = get_package_share_directory('arnav_mapping')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(maps, 'maps', 'arnav_warehouse.yaml')),
        OpaqueFunction(function=launch_setup),
    ])
