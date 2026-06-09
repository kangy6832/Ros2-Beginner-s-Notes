from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    #1.创建两个turtlesim_node 节点
    t1 = Node(package = "turtlesim", executable = "turtlesim_node")
    t2 = Node(package = "turtlesim", executable = "turtlesim_node", namespace = "t2")
    #2.让第二只乌龟掉头
    rotate = ExecuteProcess(
        #ExecuteProcess执行外部系统命令或进程
        cmd = ["ros2 action send_goal /t2/turtle1/rotate_absolute turtlesim/action/RotateAbsolute \"{'theta': 3.14}\""],
        #ros2 通信机制 具体命令 /命名空间/节点实体/具体的action名称，表示执行绝对角度旋转 包/类型/具体action名 
        #\"为了能识别引号
        #双引号只能包裹单行字符串，因此不能换行。
        output = "both",
        shell = True
    )
    #3.自实现的订阅发布实现
    pub_sub = Node(package = "cpp07_exercise", executable="exe01_pub_sub")
    #4.乌龟掉头完毕后，开始执行步骤3
    rotate_exit_event = RegisterEventHandler(
        #RegisterEventHandler 在launch描述中注册一个事件处理器，定义系统对特定事件的响应。
        event_handler=OnProcessExit( 
            #event_handler提供具体处理策略
            #OnProcessExit 一种特定的事件处理器，当指定的进程退出时触发定义的动作。
            target_action=rotate, 
            #目标进程
            on_exit=pub_sub      
            #执行操作
            
            #当rotate终止时，执行pub_sub
        )
    )
    return LaunchDescription([t1, t2, rotate, rotate_exit_event])