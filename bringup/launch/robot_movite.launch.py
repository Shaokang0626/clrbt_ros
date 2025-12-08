from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    moveit_config_pkg = FindPackageShare('clgx_arm_moveit_config')
    hardware_interface_pkg = FindPackageShare("arm_hardware_interface")
    description_pkg = FindPackageShare("arms_description")
    
    # 运动学配置文件路径
    kinematics_yaml = PathJoinSubstitution(
        [moveit_config_pkg, 'config', 'kinematics.yaml']
    )
    
    controllers_yaml = PathJoinSubstitution(
        [moveit_config_pkg, 'config', 'ros2_controllers.yaml']
    )

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            moveit_config_pkg, 'config', 'T170_ARMS.urdf.xacro'
        ]),
        ' initial_positions_file:=',
        PathJoinSubstitution([
            moveit_config_pkg, 'config', 'initial_positions.yaml'
        ])
    ])
    robot_description = {'robot_description': robot_description_content}

    # 加载运动学配置
    robot_description_kinematics = {
        'robot_description_kinematics': ParameterValue(
            kinematics_yaml, value_type=str
        )
    }

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'rsp.launch.py'])
        ),
        # 向rsp.launch.py传递运动学参数
        launch_arguments={
            'robot_description_kinematics': kinematics_yaml
        }.items()
    )

    # 配置MoveGroup启动参数，添加运动学配置
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'move_group.launch.py'])
        ),
        launch_arguments={
            'robot_description_kinematics': kinematics_yaml,
            'allow_trajectory_execution': 'true',
            'capabilities': 'move_group/MoveGroupCartesianPathService move_group/ExecuteTrajectoryAction',
            'disable_capabilities': '',
            'publish_monitored_planning_scene': 'true',
            'publish_robot_description_semantic': 'true'
        }.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'moveit_rviz.launch.py'])
        )
    )

    # 启动 ros2_control_node（硬件控制系统）
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description, 
            controllers_yaml,
            robot_description_kinematics  # 添加运动学参数
        ],
        output='both'
    )

    # 启动 环境限制节点
    scene_publisher_node = Node(
        package='clgx_arm_moveit_config',
        executable='scene_limit',
        name='scene_limit',
        output='screen'
    )

    # 控制器 spawner 节点（通过事件触发）
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    right_arm_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_position_controller'],
        output='screen'
    )

    left_arm_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_position_controller'],
        output='screen'
    )

    # 在 controller_manager 启动后再启动所有 spawner
    controller_spawner_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[
                        joint_state_broadcaster_spawner,
                        right_arm_position_controller_spawner,
                        left_arm_position_controller_spawner,
                    ]
                )
            ]
        )
    )

    return LaunchDescription([
        rsp_launch,
        controller_manager_node,
        controller_spawner_handler,  # 保证 controller_manager 启动后再加载控制器
        move_group_launch,
        rviz_launch,
        TimerAction(period=5.0, actions=[scene_publisher_node])
        # TimerAction(period=3.0, actions=[ti5_node])  # 接口节点延迟启动
    ])
