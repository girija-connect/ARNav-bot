"""
Localization: map_server + AMCL on the warehouse map.

  ros2 launch arnav_localization localization.launch.py
  ros2 launch arnav_localization localization.launch.py map:=/path/to/other_map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    loc = get_package_share_directory('arnav_localization')
    maps = get_package_share_directory('arnav_mapping')
    nav2 = get_package_share_directory('nav2_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(maps, 'maps', 'arnav_warehouse.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'localization_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': os.path.join(loc, 'config', 'localization_params.yaml'),
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_composition': 'False',     # plain processes: no shared 'nav2_container' to clash on
            }.items()),
    ])
