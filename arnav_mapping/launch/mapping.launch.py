"""
Build a new map with slam_toolbox (drive the robot with teleop).

  ros2 launch arnav_mapping mapping.launch.py

Start bringup first. Open RViz yourself to watch the map being built.
Save the result with:  ros2 run nav2_map_server map_saver_cli -f ~/ARNav_ws/src/arnav_mapping/maps/my_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mapping = get_package_share_directory('arnav_mapping')
    slam = get_package_share_directory('slam_toolbox')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam, 'launch', 'online_async_launch.py')),
            launch_arguments={'slam_params_file': os.path.join(mapping, 'config', 'slam_params.yaml'),
                              'use_sim_time': 'true'}.items()),
    ])
