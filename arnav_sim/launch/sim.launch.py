"""
Start Ignition Gazebo with the ARNAV warehouse, spawn the robot and bridge its topics to ROS 2.

  ros2 launch arnav_sim sim.launch.py                          # with the Gazebo window
  ros2 launch arnav_sim sim.launch.py gui:=false               # headless
  ros2 launch arnav_sim sim.launch.py render_engine:=ogre      # if ogre2 does not work on your GPU/VM
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD = 'arnav_warehouse'


def launch_setup(context):
    share = get_package_share_directory('arnav_sim')
    desc = get_package_share_directory('arnav_description')
    world = os.path.join(share, 'worlds', 'arnav_warehouse.sdf')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    camera = LaunchConfiguration('camera').perform(context).lower() == 'true'
    engine = LaunchConfiguration('render_engine').perform(context)

    cmd = ['ign', 'gazebo', '-r', '-v', '2', world]
    if not gui:
        cmd.insert(2, '-s')
    if engine:
        cmd += ['--render-engine-server', engine]
        if gui:
            cmd += ['--render-engine-gui', engine]
    gz = ExecuteProcess(cmd=cmd, output='screen')

    xacro_file = os.path.join(desc, 'urdf', 'arnav.urdf.xacro')
    urdf = ParameterValue(Command(['xacro ', xacro_file, ' camera:=', 'true' if camera else 'false']), value_type=str)
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': urdf, 'use_sim_time': True}])

    spawn = TimerAction(period=4.0, actions=[Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', WORLD, '-name', 'arnav', '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.03', '-Y', LaunchConfiguration('yaw')])])

    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/model/arnav/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        f'/world/{WORLD}/model/arnav/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        '/scan_front@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        '/arnav/jack/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
    ]
    if camera:
        bridge_args += ['/camera/front/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                        '/camera/front/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo']
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  arguments=bridge_args,
                  remappings=[('/model/arnav/tf', '/tf'),
                              (f'/world/{WORLD}/model/arnav/joint_state', '/joint_states')],
                  parameters=[{'use_sim_time': True}])
    return [gz, rsp, spawn, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='', description="'' (world default, ogre2) or 'ogre'"),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-9.0'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        OpaqueFunction(function=launch_setup),
    ])
