from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution
import os

def generate_launch_description():
    
    # 使用 ament_index 获取 arms_description 包的路径
    arm_description_dir = FindPackageShare('arms_description')
    urdf_path = PathJoinSubstitution([arm_description_dir, 'urdf', 'cl_arms.urdf.xacro'])

    # # 读取 URDF
    # with open(urdf_path, 'r') as f:
    #     robot_desc = f.read()
    
    robot_description = Command(['xacro ', urdf_path])
        
    # 创建节点
    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # 或者通过参数设置固定坐标系
            parameters=[{'use_sim_time': False}]
        )
    ])