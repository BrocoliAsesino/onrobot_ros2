from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import yaml
import os


def _load_yaml(package_name, relative_path):
    pkg_path = FindPackageShare(package=package_name).find(package_name)
    full_path = os.path.join(pkg_path, relative_path)
    if not os.path.exists(full_path):
        return {}
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    # Launch configurations (arguments)
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_srdf_file = LaunchConfiguration("moveit_srdf_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Gripper driver parameters
    ip_address = LaunchConfiguration("ip_address")
    port = LaunchConfiguration("port")
    slave_id = LaunchConfiguration("slave_id")
    use_dummy = LaunchConfiguration("use_dummy")

    # Build robot_description from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare(description_package),
                "urdf",
                description_file
            ]),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # robot_description_semantic (SRDF)
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare(moveit_config_package),
                "srdf",
                moveit_srdf_file
            ]),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    # # # Kinematics
    # kinematics_yaml = _load_yaml(
    #     moveit_config_package.perform(context),
    #     os.path.join("config", "kinematics.yaml")
    # )
    # robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Joint limits / planning (optional joint_limits.yaml)
    planning_yaml = _load_yaml(
        moveit_config_package.perform(context),
        os.path.join("config", "joint_limits.yaml")
    )
    if planning_yaml:
        robot_description_planning = {"robot_description_planning": planning_yaml}
    else:
        robot_description_planning = {}

    # OMPL planning pipeline config
    ompl_planning_pipeline_config = {
        "planning_pipelines": {"pipeline_names": ["ompl"]},
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision"
            ],
            "start_state_max_bounds_error": 0.1,
            "planner_configs": {
                "RRTConnectkConfigDefault": {
                    "type": "geometric::RRTConnect",
                    "range": 0.0,
                    "max_nearest_neighbors": 1000,
                }
            },
        },
    }

    # Controllers / trajectory execution config (optional)
    controllers_yaml = _load_yaml(
        moveit_config_package.perform(context),
        os.path.join("config", "moveit_controllers.yaml")
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

    # RViz config path
    rviz_config = PathJoinSubstitution([
        FindPackageShare(moveit_config_package),
        "rviz",
        rviz_config_file
    ])

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    vgc10_node = Node(
        package="vgc10driver",
        executable="vgc10_ros2_driver",
        name="vgc10_driver",
        parameters=[{
            "ip_address": ip_address,
            "port": port,
            "slave_id": slave_id,
            "use_dummy": use_dummy
        }],
        output="screen"
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            # robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(launch_rviz),
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            # robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": use_sim_time},
        ],
    )

    # # Add joint_state_publisher_gui node
    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_state_publisher_gui",
    #     parameters=[{"use_sim_time": use_sim_time}],
    # )

    ros2_controllers_yaml = _load_yaml(
        moveit_config_package.perform(context),
        os.path.join("config", "ros2_controllers.yaml")
    )
    if ros2_controllers_yaml:
        ros2_controllers = {"robot_description_planning": ros2_controllers_yaml}
    else:
        ros2_controllers = {}

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,                         # URDF
            PathJoinSubstitution([
                FindPackageShare(moveit_config_package),
                "config",
                "ros2_controllers.yaml"
            ])
        ],
        # condition=IfCondition(LaunchConfiguration("use_sim"))
    )
    
    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        # condition=IfCondition(LaunchConfiguration("use_sim"))
    )
    
    # Gripper Controller
    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["onrobot_vgc10_controller"],
        # condition=IfCondition(LaunchConfiguration("use_sim"))
    )

    return [
        robot_state_publisher_node,
        # joint_state_publisher_node,
        controller_manager,
        joint_state_broadcaster,
        # gripper_controller,
        vgc10_node,
        move_group_node,
        rviz_node,
    ]


def generate_launch_description():
    declared_arguments = []

    # Description / MoveIt packages
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="onrobot_vg_description",
            description="Package with the gripper description (URDF/Xacro).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="onrobot_vgc10_4cups_model.xacro",
            description="Xacro file for the gripper model.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="vgc10_moveit_config",
            description="MoveIt config package for the gripper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_srdf_file",
            default_value="onrobot_vgc10_model.srdf",
            description="SRDF (or SRDF xacro) file inside the MoveIt config package.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="view_robot.rviz",
            description="RViz configuration file (inside the MoveIt config package rviz/).",
        )
    )

    # Common options
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz."
        )
    )

    # Gripper driver arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ip_address",
            default_value="192.168.1.1",
            description="VGC10 Modbus TCP IP address."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "port",
            default_value="502",
            description="Modbus TCP port."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "slave_id",
            default_value="65",
            description="Modbus slave/unit ID."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_dummy",
            default_value="true",
            description="Use dummy (simulated) Modbus backend."
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
