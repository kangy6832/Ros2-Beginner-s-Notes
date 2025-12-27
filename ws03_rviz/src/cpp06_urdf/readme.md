问题：
通过 joint_state_publisher_gui 让关节运动到制定位置之后，关节存在抖动
    在初始位置和制定位置之间抖动

解决方案：不再启动 joint_state_publisher 节点
解释：
    1.joint_state_publisher 与 joint_state_publisher_gui 作用一致，都会发布非固定节点的运动信息
    2.robot_state_publisher 会订阅关节的运动信息，并生成坐标变换数据广播。
    3.joint_state_publisher 与 joint_state_publisher_gui  只有一个存在时，就会发布关节运动信息，进而就能生成坐标变换
    4.joint_state_publisher 与 joint_state_publisher_gui  都不存在，坐标树生成不了，机器人模型显示异常
    5.joint_state_publisher 与 joint_state_publisher_gui  都存在joint_state_publisher发布初始关节位姿信息，joint_state_publisher_gui发布指定位姿信息