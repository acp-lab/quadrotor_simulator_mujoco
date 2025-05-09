import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument('name', default_value='quadrotor'),
        DeclareLaunchArgument('platform_type', default_value='mujoco'),
        DeclareLaunchArgument('world_frame_id', default_value='world'),
        DeclareLaunchArgument('rate_odom', default_value='200.0'),
        DeclareLaunchArgument('rate_imu', default_value='500.0'),
        DeclareLaunchArgument('use_dq_control', default_value='false'),
    ]

    # Get values from arguments
    name = LaunchConfiguration('name')
    platform_type = LaunchConfiguration('platform_type')
    world_frame_id = LaunchConfiguration('world_frame_id')
    rate_odom = LaunchConfiguration('rate_odom')
    rate_imu = LaunchConfiguration('rate_imu')
    use_dq_control = LaunchConfiguration('use_dq_control')

    # Construct the path to the XML model using substitutions
    model_path = PathJoinSubstitution([
        get_package_share_directory('quadrotor_simulator_mujoco'),
        'model',
        platform_type,
        'drone_payload.xml'
    ])

    # Add default for model_path
    launch_args.append(
        DeclareLaunchArgument('model_path', default_value=model_path)
    )

    # Create the simulator node
    quadrotor_simulator_mujoco_node = Node(
        package='quadrotor_simulator_mujoco',
        executable='quadrotor_simulator',
        name='quadrrotor_simulator',
        namespace=name,
        output='screen',
        arguments=[LaunchConfiguration('model_path')],
        parameters=[{
        'rate_odom': rate_odom,
        'rate_imu': rate_imu,
         'world_frame_id': world_frame_id,
         'body_frame_id': name
    }]
    )

    # Controller node (flatness_controller)
    flatness_controller_node = Node(
        package='differential_flatness',
        executable='flatness_controller',
        name='flatness_controller',
        namespace=name,
        output='screen',
        parameters=[{
            'world_frame_id': world_frame_id,
            'body_frame_id': name
        }]
    )

    # Optional debug print to screen
    debug_print = LogInfo(msg=['[INFO] Using model path: ', LaunchConfiguration('model_path')])

    # Build launch description
    ld = LaunchDescription(launch_args)
    ld.add_action(debug_print)
    ld.add_action(
        GroupAction(actions=[
            quadrotor_simulator_mujoco_node
        ])
    )
    return ld