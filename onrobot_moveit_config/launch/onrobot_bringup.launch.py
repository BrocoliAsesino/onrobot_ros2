#!/usr/bin/env python3
import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from ur_moveit_config.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):
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
    include_only_plugin = LaunchConfiguration('include_only_plugin')

    _publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_controllers_file = LaunchConfiguration("moveit_controllers_file")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    
    # Get the actual onrobot_type value to determine paths
    onrobot_type_value = onrobot_type.perform(context)
    
    # Determine gripper series (vg_series or rg_series) and controller names
    if onrobot_type_value in ['vg10', 'vgc10']:
        gripper_series = 'vg_series'
        controller_file = 'vg_controllers.yaml'
        gripper_controller_name = 'vg_controller'
    elif onrobot_type_value in ['rg2', 'rg6']:
        gripper_series = 'rg_series'
        controller_file = 'rg_controllers.yaml'
        gripper_controller_name = 'finger_controller'
    else:
        raise ValueError(f"Unknown onrobot_type: {onrobot_type_value}")
    
    # Build config directory path: config/<series>/<type>/
    moveit_config_dir = os.path.join('config', gripper_series, onrobot_type_value)

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
        'num_cups:=', num_cups,
        ' ',
        'include_only_plugin:=', include_only_plugin,
        ' ',
        'name:=onrobot'
    ])
    robot_description = {'robot_description': robot_description_content}

    # Path to the controller configuration file (conditional based on gripper type)
    controller_config_file = PathJoinSubstitution([
        FindPackageShare('onrobot_drivers'),
        'config',
        controller_file
    ])
    controller_config = ParameterFile(
        controller_config_file,
        allow_substs=True
    )

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
    
    # Spawn the gripper controller (conditional based on gripper type)
    # VG grippers: vacuum_controller, RG grippers: finger_controller
    gripper_controller_spawner = Node(
        namespace=ns,
        package='controller_manager',
        executable='spawner',
        arguments=[gripper_controller_name],
        output='screen'
    )

    # Launch RViz for visualization using the config from onrobot_descriptions
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('onrobot_descriptions'),
        'rviz',
        'view_onrobot.rviz'
    ])

    # MoveIt Configuration
    # Get the moveit_config_package as string for load_yaml
    moveit_config_package_str = moveit_config_package.perform(context)
    moveit_config_package_share = FindPackageShare(moveit_config_package).find(moveit_config_package_str)
    
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), moveit_config_dir, moveit_config_file]
            ),
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(
        robot_description_semantic_content,
        value_type=str
    )}

    publish_robot_description_semantic = {
        "publish_robot_description_semantic": _publish_robot_description_semantic
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), moveit_config_dir, "kinematics.yaml"]
    )

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            moveit_config_package_str,
            os.path.join(moveit_config_dir, moveit_joint_limits_file.perform(context)),
        )
    }

    controllers_yaml = load_yaml(
        moveit_config_package_str,
        os.path.join(moveit_config_dir, moveit_controllers_file.perform(context))
    )

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        # Execution time monitoring can be incompatible with the scaled JTC
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    planning_pipelines = load_yaml(
        moveit_config_package_str,
        os.path.join(moveit_config_dir, "ompl_planning.yaml"),
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            # ompl_planning_pipeline_config,
            planning_pipelines,
            robot_description_kinematics,
            robot_description_planning,
        ],
    )

    # Construct the namespaced joint states topic for remapping
    ns_value = ns.perform(context)
    joint_states_topic = f'/{ns_value}/joint_states' if ns_value else '/joint_states'
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
        # (Internal Name in Code, Actual Name in your System)
        remappings=[
            ('/joint_states', joint_states_topic),
        ],
    )

    return [
        # Launch nodes
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_spawner,
        gripper_controller_spawner,
        rviz_node,
        move_group_node,
    ]


def generate_launch_description():
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
    declared_arguments.append(
        DeclareLaunchArgument(
            'include_only_plugin',
            default_value='false',
            description='Include only the plugin without the full robot description.',
            choices=['true', 'false'],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="onrobot_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="onrobot_vgc10_model.srdf",
            description="MoveIt SRDF description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_controllers_file",
            default_value="moveit_controllers.yaml",
            description="MoveIt controllers configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override the values from the URDF robot_description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="true",
            description="Whether to publish robot description semantic.",
        )
    )

    return LaunchDescription([
        # Declare launch arguments
        *declared_arguments,

        # Call the setup function
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    generate_launch_description()
