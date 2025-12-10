import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)
    
def generate_launch_description():

    # Launch Arguments
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("arms_moveit_config"),
            "config",
            "cl_arms.urdf.xacro",
        ]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content,value_type=str)}

    # MoveIt Configuration
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("arms_moveit_config"), "config",
            "cl_arms.srdf.xacro"
        ]),
    ])
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content,value_type=str)
    }

    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "arms_moveit_config",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    joint_limits = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("arms_moveit_config"),
            "config/joint_limits.yaml"
        ]),
        allow_substs=True,
    )

    # Planning Configuration
    ompl_planning_yaml = load_yaml("arms_moveit_config",
                                   "config/ompl_planning.yaml")

    planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": ompl_planning_yaml,
    }

    moveit_controller_manager = {
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    moveit_controllers = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("arms_moveit_config"),
            "config/moveit_controllers.yaml"
        ]),
        allow_substs=True,
    )

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controller_manager,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz
    # rviz_base = os.path.join(
    #     get_package_share_directory("arms_moveit_config"), "rviz")
    # rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline_config,
        ],
    )
    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )


    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
        ],
        output="both",
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen' 
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
        ],
    )

    
    joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
        ],
    )


    return LaunchDescription([
        # run_move_group_node,
        # rviz_node,
        robot_state_publisher,
        # ros2_control_node,
        joint_state_publisher,
        # joint_state_broadcaster_spawner,
        # joint_controller_spawner,
    ])
