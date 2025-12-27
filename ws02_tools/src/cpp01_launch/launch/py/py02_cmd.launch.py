from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description():
    #创建节点
    turtle = Node(package="turtlesim",
                executable="turtlesim_node"
                )
    
    #
    spawn = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "service call",  
            "/spawn turtlesim/srv/Spawn",  
            "\"{x: 8.0, y: 9.0, theta: 1.0, name: 'turtle2'}\""  
            #or
            #" service call /spawn turtlesim/srv/Spawn ",
            #'\"{x: 8.0, y: 9.0, theta: 1.0, name: \\'turtle2\\'}\"'
        ],
        output="both",
        shell=True #
    )
    return LaunchDescription([turtle , spawn])

"""
cmd内还可以这样写
FindExecutable(name="ros2"),
        "service", "call",
        "/spawn",
        "turtlesim/srv/Spawn",
        "{x: 8.0, y: 9.0, theta: 1.0, name: 'turtle2'}"
去除shell=True
"""