"""
Bringup: Gazebo warehouse + ARNAV robot + ROS bridge + rack stack.

  ros2 launch arnav_bringup bringup.launch.py
  ros2 launch arnav_bringup bringup.launch.py gui:=false             # headless
  ros2 launch arnav_bringup bringup.launch.py render_engine:=ogre    # if ogre2 fails
  ros2 launch arnav_bringup bringup.launch.py camera:=false          # lighter

"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(pkg, name, args=None):
    path = os.path.join(get_package_share_directory(pkg), 'launch', name)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=(args or {}).items())


def already_running():
    try:
        out = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=8).stdout
    except Exception:
        return False
    return '/robot_state_publisher' in out or '/scan_merger' in out


def launch_setup(context):
    if already_running():
        return [LogInfo(msg='ERROR: a bringup is already running (found /robot_state_publisher or /scan_merger). '
                            'Stop it first: ~/ARNav_ws/src/arnav_bringup/scripts/stop_all.sh')]
    return [
        include('arnav_sim', 'sim.launch.py', {
            'gui': LaunchConfiguration('gui'), 'camera': LaunchConfiguration('camera'),
            'render_engine': LaunchConfiguration('render_engine')}),
        # the rack stack needs /scan_front and odom, so give the robot time to spawn first
        TimerAction(period=8.0, actions=[include('arnav_rack', 'rack_stack.launch.py')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='open the Gazebo window'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
