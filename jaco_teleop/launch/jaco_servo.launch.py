# this replaces jaco_ros2_teleop and jaco_base (this is basically doign the same thing but teleop with the camera instea dof the joystick and snp and ha)

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchConfiguration([
        IncludeLaunchDescription(
            launch_description_source=XMLLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('argallab_jaco_moveit'), 'launch', 'demo.launch.py'])),
        ),
        
        # launch the jaco_teleop node
        Node(
            package='jaco_teleop',
            executable='jaco_servo_real',
            name='jaco_servo_real',
            output='screen',
            # parameters=[
            #     {'use_realsense': LaunchConfiguration('use_realsense')},
            #     {'run_jaco_teleop': LaunchConfiguration('run_jaco_teleop')},
            #     {'rviz_file': LaunchConfiguration('rviz_file')},
            # ],
        ),
    ])