import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

def generate_launch_description():
    # Package paths
    pkg_share = get_package_share_directory('snot_simulation')
    urdf_file = os.path.join(pkg_share, 'urdf', 'snot_bot_sim_urdf.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'empty_world.world')

    # Verify if the URDF file exists
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    # Launch configuration variables
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_joint_state_pub_cmd = DeclareLaunchArgument(
        'use_joint_state_pub',
        default_value='True',
        description='Whether to start the joint state publisher')

    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_description_param = {'robot_description': robot_desc}

    # Start the robot_state_publisher node with delay to ensure Gazebo is ready
    start_robot_state_publisher_cmd = TimerAction(
        period=2.0,  # Delay in seconds
        actions=[Node(
            condition=IfCondition(use_robot_state_pub),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description_param]
        )]
    )

    # Start the joint_state_publisher node
    start_joint_state_publisher_cmd = Node(
        condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        arguments=[urdf_file]
    )

    # Log URDF file path and content
    log_urdf_path = LogInfo(msg=f'URDF file path: {urdf_file}')
    log_urdf_content = LogInfo(msg=f'URDF content: {robot_desc[:500]}...')

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items(),
    )

    # Spawn the robot in Gazebo with a delay to ensure Gazebo is ready
    delayed_spawn_entity = TimerAction(
        period=3.0,  # Delay to ensure Gazebo is ready
        actions=[Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=['-entity', 'snot_bot', '-topic', '/robot_description', '-z', '0.5'],  # Spawns robot 0.5m above ground
            output='screen'
        )]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_joint_state_pub_cmd)

    # Add logging actions
    ld.add_action(log_urdf_path)
    ld.add_action(log_urdf_content)

    # Add nodes and actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(gazebo)
    ld.add_action(delayed_spawn_entity)

    return ld
