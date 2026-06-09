cmake_minimum_required(VERSION 3.8)
project(cpp_service_01)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(base_interface_demo REQUIRED)
add_executable(demo01_server src/demo01_server.cpp)
ament_target_dependencies(
  demo01_server
  "rclcpp"
  "base_interface_demo"
)

add_executable(demo01_client src/demo01_client.cpp)
ament_target_dependencies(
  demo01_client
  "rclcpp"
  "base_interface_demo"
)

install(TARGETS 
  demo01_server
  demo01_client 
  DESTINATION lib/${PROJECT_NAME})

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
