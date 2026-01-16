from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def noisy_controller(context, *args, **kwargs):
    wheel_radius = float(LaunchConfiguration('wheel_radius').perform(context))
    wheel_separation = float(LaunchConfiguration('wheel_separation').perform(context))
    wheel_radius_error = float(LaunchConfiguration('wheel_radius_error').perform(context))
    wheel_separation_error = float(LaunchConfiguration('wheel_separation_error').perform(context))
    use_python = LaunchConfiguration('use_python').perform(context)
    
    noisy_controller_py = Node(
        package='bumperbot_controller',
        executable='noisy_controller.py',
        name='noisy_controller',
        parameters=[{
            'wheel_radius': wheel_radius + wheel_radius_error,
            'wheel_separation': wheel_separation + wheel_separation_error
        }],
        condition = IfCondition(use_python)
    )
    
    return [noisy_controller_py]
        

def generate_launch_description():
    
    use_python_arg = DeclareLaunchArgument(
        'use_python',
        default_value='true',
        description='Whether to use the Python-based Arduino interface'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.033',
        description='Radius of the wheels'
    )
    
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.17',
        description='Separation between the wheels'
    )
    
    use_simple_controller_arg = DeclareLaunchArgument(
        'use_simple_controller',
        default_value='true',
        description='Whether to use the simple velocity controller'
    )
    
    wheel_radius_error_arg = DeclareLaunchArgument(
        'wheel_radius_error',
        default_value='0.005',
        description='Error in wheel radius'
    )
    
    wheel_separation_error_arg = DeclareLaunchArgument(
        'wheel_separation_error',
        default_value='0.02',
        description='Error in wheel separation'
    )
    
    use_python = LaunchConfiguration('use_python')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')
    use_simple_controller = LaunchConfiguration('use_simple_controller')
    
    
    
    joint_state_broadcaster_spawnner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            ]
    )
    
    wheel_controller_spawnner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "bumperbot_controller",
            "--controller-manager",
            "/controller_manager",
            ],
        condition=UnlessCondition(use_simple_controller)
    )   
    
    simple_controller_spawner = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager",
                ]
            )
        ]
    )
    
    simple_controller_py = Node(
        package='bumperbot_controller',
        executable='simple_controller.py',
        name='simple_controller',
        parameters=[{
            'wheel_radius': wheel_radius,
            'wheel_separation': wheel_separation
        }],
        condition = IfCondition(use_python)
    )
    
    Noisy_controller_launch = OpaqueFunction(
        function=noisy_controller
    )
    
    
    
    
    return LaunchDescription([
        use_python_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        use_simple_controller_arg,
        joint_state_broadcaster_spawnner,
        wheel_controller_spawnner,
        simple_controller_spawner,
        simple_controller_py,
        Noisy_controller_launch,
        
        
    ])