import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_mobile_manipulator_body = FindPackageShare(package='mobile_manipulator_body').find('mobile_manipulator_body')
    urdf_path = os.path.join(pkg_mobile_manipulator_body, 'urdf', 'robot_arm.urdf')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': Command(['xacro ', urdf_path])  # Adjust if not using xacro
        }]
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': Command(['xacro ', urdf_path])  # Adjust if not using xacro
        }],
        output='screen'
    )

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

    # Static transforms
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_arm_base_to_map',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'map', 'arm_base']  # Adjust values based on your setup
    )

    static_transform_bicep = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_bicep',
        output='screen',
        arguments=['0.0', '0.0', '0.1', '0', '0', '0', 'arm_base', 'bicep']  # Adjust values as needed
    )

    static_transform_bottom_wrist = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_bottom_wrist',
        output='screen',
        arguments=['0.0', '0.0', '0.1', '0', '0', '0', 'bicep', 'bottom_wrist']  # Changed parent to bicep
    )

    static_transform_elbow = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_elbow',
        output='screen',
        arguments=['0.0', '0.0', '0.15', '0', '0', '0', 'bottom_wrist', 'elbow']
    )

    static_transform_top_wrist = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_top_wrist',
        output='screen',
        arguments=['0.0', '0.0', '0.15', '0', '0', '0', 'elbow', 'top_wrist']
    )

    static_transform_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_world',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'world', 'map']  # Adjust as needed
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', os.path.join(pkg_mobile_manipulator_body, 'rviz', 'robot_arm.rviz')]
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
        controller_manager_node,
        load_arm_control_node,
        load_joint_state_controller_node,
        static_transform_node,
        static_transform_bicep,
        static_transform_bottom_wrist,
        static_transform_elbow,
        static_transform_top_wrist,
        static_transform_world,
        rviz_node
    ])

