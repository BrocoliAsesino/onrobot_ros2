#!/usr/bin/env python3
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

def generate_launch_description():
    # Launch configuration variables
    onrobot_type = LaunchConfiguration('onrobot_type')
    connection_type = LaunchConfiguration('connection_type')
    device = LaunchConfiguration('device')
    ip_address = LaunchConfiguration('ip_address')
    port = LaunchConfiguration('port')
    prefix = LaunchConfiguration('prefix')
    ns = LaunchConfiguration('ns')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_rsp = LaunchConfiguration('launch_rsp')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    num_cups = LaunchConfiguration('num_cups')

    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'onrobot_type',
            description='Type of OnRobot gripper.',
            choices=['vgc10', 'vg10', 'rg2', 'rg6'],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'connection_type',
            description='Connection type for the OnRobot gripper. TCP for the Control Box. Serial for the UR Tool I/O (RS485).',
            choices=['serial', 'tcp'],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'device',
            default_value='/tmp/ttyUR',
            description='Device name for the serial connection. Only used when connection_type is serial.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ip_address',
            default_value='192.168.1.1',
            description='IP address for the TCP connection. Only used when connection_type is tcp.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'port',
            default_value='502',
            description='Port for the TCP connection. Only used when connection_type is tcp.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix for joint names (useful for multi-robot setups).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ns',
            default_value='onrobot',
            description='Namespace for the nodes. Useful for separate gripper and robot control setups.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz for visualization.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rsp',
            default_value='true',
            description='Launch robot state publisher.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware interface for testing.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'num_cups',
            default_value='4',
            description='Number of suction cups for VGC10 gripper (1 or 4). Only used for vgc10 type.',
            choices=['1', '4'],
        )
    )

    # Path to the xacro file in the onrobot_descriptions package
    xacro_file = PathJoinSubstitution([
        FindPackageShare('onrobot_descriptions'),
        'urdf',
        'onrobot.urdf.xacro'
    ])

    # Process the xacro to generate the robot description (URDF)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file,
        ' ',
        'onrobot_type:=', onrobot_type,
        ' ',
        'connection_type:=', connection_type,
        ' ',
        'device:=', device,
        ' ',
        'ip_address:=', ip_address,
        ' ',
        'port:=', port,
        ' ',
        'prefix:=', prefix,
        ' ',
        'use_fake_hardware:=', use_fake_hardware,
        ' ',
        ' um_cups:=', num_cups,
        ' ',
        'name:=onrobot'
    ])
    robot_description = {'robot_description': robot_description_content}

    # Path to the controller configuration file (using ParameterFile to load YAML)
    # Select configuration based on gripper type
    def get_controller_config():
        gripper_type_value = LaunchConfiguration('onrobot_type')
        # VG grippers use vg_controllers.yaml, RG grippers use rg_controllers.yaml
        # Note: This is evaluated at launch time based on the argument
        return PathJoinSubstitution([
            FindPackageShare('onrobot_drivers'),
            'config',
            # Default to vg_controllers, but will need conditional loading
            'vg_controllers.yaml'
        ])
    
    controller_config_file = get_controller_config()
    controller_config = ParameterFile(controller_config_file, allow_substs=True)

    # Launch the ros2_control node
    ros2_control_node = Node(
        namespace=ns,
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen'
    )

    # Launch the robot state publisher
    robot_state_publisher_node = Node(
        namespace=ns,
        package='robot_state_publisher',
        condition=IfCondition(launch_rsp),
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='both'
    )

    # Spawn the joint state and gripper controllers
    joint_state_spawner = Node(
        namespace=ns,
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # Controller name depends on gripper type
    # VG grippers: vacuum_controller, RG grippers: finger_controller
    # Note: In practice, you might want to use PythonExpression or OpaqueFunction
    # to conditionally set this based on onrobot_type
    gripper_controller_spawner = Node(
        namespace=ns,
        package='controller_manager',
        executable='spawner',
        arguments=['vacuum_controller'],  # TODO: Make this conditional
        output='screen'
    )

    # Launch RViz for visualization using the config from onrobot_descriptions
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('onrobot_descriptions'),
        'rviz',
        'view_onrobot.rviz'
    ])
    rviz_node = Node(
        namespace=ns,
        package='rviz2',
        condition=IfCondition(launch_rviz),
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        # Declare launch arguments
        *declared_arguments,

        # Launch nodes
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_spawner,
        gripper_controller_spawner,
        rviz_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
