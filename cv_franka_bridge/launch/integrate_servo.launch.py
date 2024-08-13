import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, Shutdown, DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command, AndSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_fake_hardware", default_value="true",
                                  description="whether or not to use fake hardware."),
            DeclareLaunchArgument(name="use_rviz", default_value="true",
                                  description="whether or not to use rviz."),
            DeclareLaunchArgument(name="robot_ip", default_value="dont-care",
                                  description="IP address of the robot"),
            DeclareLaunchArgument(name="use_realsense", default_value="true",
                                  description="whether or not to use realsense camera."),
            DeclareLaunchArgument(name="run_jaco_teleop", default_value="true",
                                  description="whether or not to run franka teleop."),
            DeclareLaunchArgument(name="rviz_file", default_value="integrate_servo.rviz",
                                  description="rviz file to use."),
            # IncludeLaunchDescription(
            #     XMLLaunchDescriptionSource([PathJoinSubstitution(
            #         [FindPackageShare('jaco_ros2_interaction'), 'jaco_base.launch.xml'])]),
            #     condition=IfCondition(LaunchConfiguration("run_jaco_teleop")),
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution(
                    [FindPackageShare('handcv'), 'launch', 'camera.launch.py'])]),
                launch_arguments={'use_realsense': LaunchConfiguration("use_realsense")}.items(),
                condition=IfCondition(LaunchConfiguration("use_realsense"))
            ),
            Node(
                package="cv_franka_bridge",
                executable="cv_franka_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_realsense")),
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '-1.5708', '--pitch', '0', '--roll', '-1.5708', '--frame-id', 'j2s7s300_link_base', '--child-frame-id', 'camera_link']
            ),
            
            Node(
                package='jaco_ros2_teleop',
                executable='hybrid_uservel_to_cartesianvel',
                output='screen',
                condition=IfCondition(LaunchConfiguration("run_jaco_teleop")),
            )
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource([PathJoinSubstitution(
            #         [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
            #     condition=IfCondition(LaunchConfiguration("run_franka_teleop")),
            # ),
        ]
    )
