from pkg_resources import declare_namespace
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    #声明参数
    decl_bg_r=DeclareLaunchArgument(name="background_r",default_value='255')
    decl_bg_g=DeclareLaunchArgument(name='background_g',default_value='255')
    decl_bg_b=DeclareLaunchArgument(name='background_b',default_value='255')
    
    #获取参数
    turtle=Node(package='turtlesim',
                executable='turtlesim_node',
                parameters=[{"background_r":LaunchConfiguration("background_r"),
                            "background_g":LaunchConfiguration("background_g"),
                            'background_b':LaunchConfiguration('background_b')
                            }]
                )
    return LaunchDescription([decl_bg_r,decl_bg_b,decl_bg_g])