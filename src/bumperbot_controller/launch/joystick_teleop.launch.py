from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick_driver',
        output='screen',
        parameters=[{os.path.join(get_package_share_directory('bumperbot_controller'), 'config', 'joy_config.yaml')}]
    )
    
    joy_teleop = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joystick_teleop',
        output='screen',
        parameters=[{os.path.join(get_package_share_directory('bumperbot_controller'), 'config', 'joy_teleop.yaml')}]
    )
    
    
    
    return LaunchDescription([
        joy_node,
        joy_teleop
        
    ])