/*
${workspaceFolder}/**
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
 */
#include "rclcpp/rclcpp.hpp"
#include "interface_demo/msg/student.hpp"
using interface_demo::msg::Student;

class TopicSubStudent : public rclcpp::Node{
    private:
        rclcpp::Subscription<Student>::SharedPtr subscription_;
    public:
        TopicSubStudent():Node("cpp02_lisener"){
            subscription_ = this->create_subscription<Student>("topic_02" , 10 ,
                std::bind(&TopicSubStudent::CinStudent , this , std::placeholders::_1));
        }
    private:
        void CinStudent(const Student& meg) const {
            RCLCPP_INFO(this->get_logger(), "subscription: %s, %d, %.2f", meg.name.c_str(), meg.age, meg.height);
        }
};

int main(int argc , char** argv){
    rclcpp::init(argc , argv);
    auto node = std::make_shared<TopicSubStudent>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
