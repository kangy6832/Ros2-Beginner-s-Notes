'''
chapt1.ws03_rviz.src.cpp06_urdf.launch.display.launch 的 Docstring
核心：
    1.启动robot_state_publisher节点，该节点以参数方式加载urdf文件；
    2.启动rviz2节点
优化：
    1.添加joint_state_publisher节点（用于非固定节点）
    2.设置rviz2的默认配置文件
    3.程序可以动态传入urdf文件，将urdf文件封装为参数
'''
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command 
from launch.substitutions import LaunchConfiguration

from launch_ros.parameter_descriptions import ParameterValue

#与操作系统交互
import os

#获取功能包下share目录路径
from ament_index_python.packages import get_package_share_directory


def generate_launch_description ():
    #返回功能包下share的完整路径
    cpp06_urdf_dir = get_package_share_directory("cpp06_urdf")
    #路径字符串拼接
    default_model_path = os.path.join(cpp06_urdf_dir, "urdf/urdf", "demo01_helloworld.urdf")
    default_rviz_config_path = os.path.join(cpp06_urdf_dir, "rviz", "urdf.rviz")
    
    # 如果rviz目录不存在或没有配置文件，可以创建或使用临时配置
    if not os.path.exists(default_rviz_config_path):
        default_rviz_config_path=""
    #定义能接受外部输入的可配置参数,并设置默认值（default_value）
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Path to URDF/XACRO model file"
    )
    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=default_rviz_config_path,
        description="Path to RViz configuration file"
    )
    
    #加载机器人模型
    model_path = LaunchConfiguration("model")
    robot_description = ParameterValue(Command([
        "xacro ",model_path
    ]))
    # 1.启动 robot_state_publisher 节点并以参数方式加载 urdf 文件
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        #参数
        parameters=[{
            "robot_description":robot_description
        }]
    )
    
    # 2.启动 joint_state_publisher 节点发布非固定关节状态
    # 优化1
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher"
    )
    
    # rviz2 节点
    rviz2 = Node(
        package="rviz2", 
        executable="rviz2",
        arguments=["-d",LaunchConfiguration("rviz_config")]
    )
    
    
    return LaunchDescription([model_arg,rviz_config_arg, robot_state_publisher, joint_state_publisher, rviz2])