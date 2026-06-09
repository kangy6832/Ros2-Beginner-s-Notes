cmake_minimum_required(VERSION 3.8)
project(cpp07_exercise)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(base_interface_demo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(exe01_pub_sub src/exe01_pub_sub.cpp)
target_include_directories(exe01_pub_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(exe01_pub_sub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  exe01_pub_sub
  "rclcpp"
  "turtlesim"
  "base_interface_demo"
  "geometry_msgs"
  "rclcpp_action"
)

add_executable(exe02_server src/exe02_server.cpp)
ament_target_dependencies(
  exe02_server
  "rclcpp"
  "turtlesim"
  "base_interface_demo"
  "geometry_msgs"
)

add_executable(exe03_client src/exe03_client.cpp)
ament_target_dependencies(
  exe03_client 
  "rclcpp"
  "turtlesim"
  "base_interface_demo"
  "geometry_msgs"
)

add_executable(exe04_action_server src/exe04_action_server.cpp)
ament_target_dependencies(
  exe04_action_server
  "rclcpp"
  "turtlesim"
  "base_interfaces_demo"
  "geometry_msgs"
  "rclcpp_action"
)

add_executable(exe05_action_client src/exe05_action_client.cpp)
ament_target_dependencies(
  exe05_action_client
  "rclcpp"
  "turtlesim"
  "base_interfaces_demo"
  "geometry_msgs"
  "rclcpp_action"
)

add_executable(exe06_param src/exe06_param.cpp)
ament_target_dependencies(
  exe06_param
  "rclcpp"
  "turtlesim"
)

install(TARGETS 
  exe01_pub_sub
  exe02_server 
  exe03_client
  exe04_action_server
  exe05_action_client 
  exe06_param
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
