BUG说明：
当海龟1向后运动时，海龟2仍向前运动

原因：
turtlesim/msg/Pose中linear的计算方法是 sqrt(lin_vel_x_ * lin_vel_x_ + lin_vel_y_ * lin_vel_y_) 始终为正

解决办法：
下载并修改源代码为 lin_vel_x_
编译是在最后加上 --allow-overriding turtlesim（允许覆盖系统原代码）