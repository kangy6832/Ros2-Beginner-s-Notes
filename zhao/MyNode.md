export ROS_DOMAIN_ID=6  改变域ID为6,只在当前终端中有效。
echo $ROS_DOMAIN_ID 查看环境变量
env | grep ROS 查看所有ros有关环境变量
echo "export ROS_DOMAIN_ID=6" >> ~/.bashrc 设置永久id

