import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the package directory
    bringup_dir = get_package_share_directory('snot_simulation')
    urdf_file = os.path.join(bringup_dir, 'urdf', 'snot_bot_sim_urdf.urdf')

    # Verify if the URDF file exists
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    # Declare launch arguments
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='true',
        description='Whether to start the robot state publisher')

    declare_use_joint_state_pub_cmd = DeclareLaunchArgument(
        'use_joint_state_pub',
        default_value='true',
        description='Whether to start the joint state publisher')

    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_description_param = {'robot_description': robot_desc}

    # Start the robot_state_publisher node
    start_robot_state_publisher_cmd = TimerAction(
        period=2.0,  # Delay in seconds
        actions=[Node(
            condition=IfCondition(LaunchConfiguration('use_robot_state_pub')),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description_param]
        )]
    )

    # Start the joint_state_publisher node
    start_joint_state_publisher_cmd = Node(
        condition=IfCondition(LaunchConfiguration('use_joint_state_pub')),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        arguments=[urdf_file]
    )

    # Log URDF file path and content
    log_urdf_path = LogInfo(msg=f'URDF file path: {urdf_file}')
    log_urdf_content = LogInfo(msg=f'URDF content: {robot_desc[:500]}...')

    # Create the launch description
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_joint_state_pub_cmd)

    # Add logging actions
    ld.add_action(log_urdf_path)
    ld.add_action(log_urdf_content)

    # Add nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)

    return ld
