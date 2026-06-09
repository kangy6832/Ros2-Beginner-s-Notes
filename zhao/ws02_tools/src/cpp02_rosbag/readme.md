ros2 pkg create cpp02_rosbag --build-type ament_cmake --dependencies rclcpp rosbag2_cpp geometry_msgs

add_executable(demo01_writer sc/demo01_writer.cpp)
ament_target_dependencies(
  demo01_writer
  "rclcpp"
  "rosbag2_cpp"
  "geometry_msgs"
)
add_executable(demo02_reader sc/demo0_reader.cpp)
ament_target_dependencies(
  demo02_reader
  "rclcpp"
  "rosbag2_cpp"
  "geometry_msgs"
)
install(TARGETS 
  demo01_writer 
  demo02_reader
  DESTINATION lib/${PROJECT_NAME})

另外，建议使用命令行实现
convert  给定一个bag文件，写出一个新的具有不同配置的bag文件
info     输出bag文件的相关信息
list     输出可用插件信息
play     回放bag问件
record   录制ag文件
reindex  重建bag的元数据文件

建议通过-h深度学习


