from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.substitutions import FindExecutable
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_desription():
    turtle=Node(package='turtlesim', executable='turtlesim_node')
    spawn=ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'service call',
            '/spaen turtlesim/srv/Spawn',
            "\"{x:8.0, y:1.0, theta:1.0, name:'turtle2'}\""
        ],
        output='both',
        shell=True
    )
    
    start_event=RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=turtle,
            on_start=spawn
        )
    )
    
    exit_event=RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=turtle,
            on_exit=[LogInfo(msg='turtlesim_node exit')]
        )
    )
    
    return LaunchDescription([turtle,start_event,exit_event])