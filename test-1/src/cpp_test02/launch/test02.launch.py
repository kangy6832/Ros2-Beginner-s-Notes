from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    client = Node(
        package="cpp_test02",
        executable="test02_client",
        output="screen"
    )
    
    server = Node(
        package="cpp_test02",
        executable="test02_server",
        output="screen"
    )
    return LaunchDescription([client, server])
    