from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
    turtle1 = Node(package='turtlesim',executable="turtlesim_node", name="t1")
    turtle2=Node(package='turtlesim',executable='turtlesim_node', name='t2')
    turtle3=Node(package='turtlesim', executable='turtlesim_node', name='t3')
    g1=GroupAction(actions=[PushRosNamespace(namespace='g1'), turtle1 ,turtle2])
    g2=GroupAction(actions=[PushRosNamespace(namespace="g2"),turtle3])
    
    return LaunchDescription([g1,g2])