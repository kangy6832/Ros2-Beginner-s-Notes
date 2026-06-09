from launch import LaunchDescription
from launch_ros.actions import Node 
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description ():
    turtle1 = Node ( package="turtlesim", #功能包
                    executable="turtlesim_node", #可执行文件
                    namespace="ns_1", #命名空间
                    name="t1",  #节点名称
                    exec_name="turtle_label", #流程标签
                    respawn=True  #关闭节点后，允许自动重启
                    )
    
    #拼接yaml文件路径
    config_path = os.path.join(get_package_share_directory("cpp01_launch"),"config","t2.yaml")
    turtle2 = Node (package="turtlesim",
                    executable="turtlesim_node",
                    name="t2",
                    #从config文件夹下的yaml文件下载参数
                    parameters=[config_path]
                    )
    
    turtle3 = Node (package="turtlesim",
                    executable="turtlesim_node",
                    name="t3",
                    remappings=[("/turtle1cmd_vel","/cmd_vel")] #话题重映射
                    )
    
    #创建一个 rviz2 节点，加载了 rviz2 相关的配置文件，该配置文件可以先启动 rviz2 ，配置完毕后，保存到 config目录并命名为my.rviz
    rviz = Node (package="rviz2",
                executable="rviz2",
                #节点启动时传参
                arguments=["-d",config_path]
                )
    
    turtle4 = Node (package="turtlesim",
                    executable="turtlesim_node",
                    ros_arguments=["--remap","__ns:=/t4_ns","--remap","__node:=t4"]
                    )
    
    return LaunchDescription([turtle1,turtle2])

    