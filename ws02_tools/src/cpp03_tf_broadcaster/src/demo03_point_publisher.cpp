/* ${workspaceFolder}/**
/opt/ros/humble/include/**
/opt/ros/humble/include/std_msgs/**
/opt/ros/humble/include/rclcpp/**
/opt/ros/humble/include/rcl/**
/opt/ros/humble/include/rclcpp_action/**
/opt/ros/humble/include/rclcpp
/opt/ros/humble/include/rcl
/opt/ros/humble/include/std_msgs
/opt/ros/humble/include/rcutils
/opt/ros/humble/include/rmw
/opt/ros/humble/include/rosbag2_cpp
${workspaceFolder}/chapt1/ws01/build/interface_demo/rosidl_generator_cpp
/opt/ros/humble/include/rosidl_runtime_cpp
/opt/ros/humble/include/geometry_msgs
/opt/ros/humble/include/turtlesim
/opt/ros/humble/include/rosidl_runtime_c
/opt/ros/humble/include/builtin_interfaces */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
using namespace std::chrono_literals;
class MinimalPointPublisher : public rclcpp::Node{
    private:
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        double_t x;
    public:
        MinimalPointPublisher():Node("minimal_point_publisher"), x(0.1){
            point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point", 10);
            timer_ = this->create_wall_timer(100ms, std::bind(&MinimalPointPublisher::on_timer, this));
        }
    private:
        void on_timer(){
            geometry_msgs::msg::PointStamped point;
            point.header.frame_id = "laser";
            point.header.stamp = this->now();
            x += 0.04;
            point.point.x = x;
            point.point.y = 0.0;
            point.point.z = 0.1;
            point_pub_->publish(point);
            RCLCPP_INFO(
                this->get_logger(), 
                "发布点: (%.2f, %.2f, %.2f) 坐标系: %s",
                point.point.x, point.point.y, point.point.z,
                point.header.frame_id.c_str()
            );
        }
};
int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPointPublisher>());
    rclcpp::shutdown();
    return 0;
}