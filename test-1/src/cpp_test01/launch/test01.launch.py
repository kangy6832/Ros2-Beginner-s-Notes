from launch import LaunchDescription
from launch_ros.actions import Node 


def generate_launch_description():
    test01 = Node(
        package="cpp_test01",
        executable="test01_rosbag2_reade",
        output='screen',
    )
    return LaunchDescription([test01])

    