import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('arnav_rack'), 'config', 'rack.yaml')
    nodes = [Node(package='arnav_rack', executable=e, name=e, output='screen', parameters=[cfg])
             for e in ('scan_merger', 'rack_detector', 'jack_controller', 'rack_docking')]
    # Simulated RFID reader. On the real robot start with  sim_rfid:=false  and run your reader driver
    # (publishes arnav_interfaces/RfidTag on /rfid/tag).
    nodes.append(Node(package='arnav_rack', executable='rfid_reader_sim', name='rfid_reader_sim', output='screen',
                      parameters=[cfg], condition=IfCondition(LaunchConfiguration('sim_rfid'))))
    return LaunchDescription([DeclareLaunchArgument('sim_rfid', default_value='true')] + nodes)
