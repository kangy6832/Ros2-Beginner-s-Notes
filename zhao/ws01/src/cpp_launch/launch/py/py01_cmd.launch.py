import launch 
import launch_ros

def generate_launch_description():
    cpp_launch_topic_01 = launch_ros.actions.Node(
        package="cpp_topic",
        executable="cpp02_talker",
        output="screen",
    )
    cpp_launch_topic_02 = launch_ros.actions.Node(
        package="cpp_topic",
        executable="cpp02_lisener",
        output='screen',
    )
    launch_description = launch.LaunchDescription([ #注意隐秘的中括号
        cpp_launch_topic_01,
        cpp_launch_topic_02
    ])
    return launch_description