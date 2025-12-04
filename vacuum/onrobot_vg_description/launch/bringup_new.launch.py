#!/usr/bin/env python3
#
# View OnRobot VGC10 gripper in RViz with basic MoveIt components.
#
# Minimal planning configuration (OMPL) is inlined so this file works
# even if some optional config YAMLs are absent.

import os
import yaml

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
import launch_ros
from launch_ros.parameter_descriptions import ParameterValue


def _pkg_share(package_name: str) -> str:
    return FindPackageShare(package=package_name).find(package_name)


def _load_yaml(package_name: str, relative_path: str):
    try:
        pkg_path = _pkg_share(package_name)
        full_path = os.path.join(pkg_path, relative_path)
        if not os.path.exists(full_path):
            return {}
        with open(full_path, "r") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


def generate_launch_description():
    # Packages
    description_pkg = "onrobot_vg_description"
    moveit_config_pkg = "vgc10_moveit_config"

    description_share = _pkg_share(description_pkg)
    moveit_config_share = _pkg_share(moveit_config_pkg)

    # Default paths
    default_model_path = os.path.join(
        description_share, "urdf", "onrobot_vgc10_4cups_model.xacro"
    )
    srdf_model_path = os.path.join(
        moveit_config_share, "srdf", "onrobot_vgc10_model.srdf"
    )
    default_rviz_config_path = os.path.join(description_share, "rviz", "view_urdf.rviz")

    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "model",
            default_value=default_model_path,
            description="Absolute path to gripper Xacro/URDF",
        ),
        DeclareLaunchArgument(
            "rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to RViz2 config file",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock if available",
        ),
    ]

    # Robot description (URDF from xacro)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration("model"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Semantic description (SRDF)
    if os.path.exists(srdf_model_path):
        with open(srdf_model_path, "r") as f:
            robot_description_semantic = {
                "robot_description_semantic": f.read()
            }
    else:
        robot_description_semantic = {"robot_description_semantic": ""}

    # Optional configs (load gracefully)
    kinematics_yaml = _load_yaml(moveit_config_pkg, os.path.join("config", "kinematics.yaml"))
    joint_limits_yaml = _load_yaml(moveit_config_pkg, os.path.join("config", "joint_limits.yaml"))
    ompl_planning_yaml = _load_yaml(moveit_config_pkg, os.path.join("config", "ompl_planning.yaml"))
    # Planning pipeline (OMPL minimal config)
    ompl_planning_pipeline_config = {
        # REQUIRED: list of enabled pipelines (now under .pipeline_names)
        "planning_pipelines": {"pipeline_names": ["ompl"]},
        "default_planning_pipeline": "ompl",

        # OMPL namespace
        "ompl": {
            # NOTE: plural + list in Jazzy
            "planning_plugins": ["ompl_interface/OMPLPlanner"],

            "request_adapters": [
            "default_planning_request_adapters/ResolveConstraintFrames",
            "default_planning_request_adapters/ValidateWorkspaceBounds",
            "default_planning_request_adapters/CheckStartStateBounds",
            "default_planning_request_adapters/CheckStartStateCollision"
            ],

            # ✅ Response adapters (post-processing / timing)
            # Pick ONE of these lines:
            # "response_adapters": [
            #     "default_planning_response_adapters/AddTimeOptimalParameterization"
            #     # or: "default_planning_response_adapters/AddRuckigTrajectorySmoothing"
            # ],
            "start_state_max_bounds_error": 0.1,

            # (optional) planner configs live under 'planner_configs'
            "planner_configs": {
                "RRTConnectkConfigDefault": {
                    "type": "geometric::RRTConnect",
                    "range": 0.0,
                    "max_nearest_neighbors": 1000,
                }
            },
        },
    }


    # ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Robot planning parameters (joint limits + kinematics if present)
    robot_description_planning = {}
    if joint_limits_yaml:
        robot_description_planning["robot_description_planning"] = joint_limits_yaml
    if kinematics_yaml:
        robot_description_planning["robot_description_kinematics"] = kinematics_yaml

    # Controllers (optional)
    controllers_yaml = _load_yaml(
        moveit_config_pkg, os.path.join("config", "moveit_controllers.yaml")
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
    }

    # Trajectory execution parameters
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    # Planning scene monitor
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    # Nodes
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    move_group_node = launch_ros.actions.Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            
        ],
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    vgc10_node = launch_ros.actions.Node(
        package="vgc10driver",
        executable="vgc10_ros2_driver",
        name="vgc10_driver",
        parameters=[{
            'ip_address': '192.168.1.1',
            'port': 502,
            'slave_id': 65,
            'use_dummy': True
        }],
        output="screen"
    )

    return launch.LaunchDescription(
        declared_arguments
        + [
            vgc10_node,
            robot_state_publisher_node,
            joint_state_publisher_node,
            move_group_node,
            rviz_node,
        ]
    )