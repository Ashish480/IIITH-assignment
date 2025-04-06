import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define paths to required files and packages
    pkg_mobile_manipulator_body = FindPackageShare(package='mobile_manipulator_body').find('mobile_manipulator_body')
    urdf_path = os.path.join(pkg_mobile_manipulator_body, 'urdf', 'robot_arm.urdf')
    arm_control_config = os.path.join(pkg_mobile_manipulator_body, 'config', 'arm_control.yaml')
    joint_state_control_config = os.path.join(pkg_mobile_manipulator_body, 'config', 'joint_state_controller.yaml')
    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
    )

    # Define launch arguments (optional, for configurability)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )

    # Include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # Define the node to spawn the URDF model in Gazebo
    spawn_urdf_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_urdf',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot_arm'
        ]
    )

    # Nodes for controllers
    load_arm_control_node = Node(
        package='controller_manager',
        executable='spawner',
        name='arm_controller_spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    load_joint_state_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_controller_spawner',
        arguments=['joint_state_controller'],
        output='screen'
    )

    # Node for robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': Command(['xacro ', urdf_path])
        }],
        output='screen'
    )

    # Create LaunchDescription and add all actions
    return LaunchDescription([
        declare_use_sim_time,
        gazebo_launch,
        spawn_urdf_node,
        load_arm_control_node,
        load_joint_state_controller_node,
        robot_state_publisher_node
    ])

