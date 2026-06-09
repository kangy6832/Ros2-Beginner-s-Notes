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
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

class TopicPublisher : public rclcpp::Node{
    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

    public:
        TopicPublisher():Node("TopicPublisher") , count_(0){
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic_std" , 10);
            timer_ = this->create_wall_timer(1000ms , std::bind(&TopicPublisher::CoutNum , this ));
        }

    private:
        void CoutNum (){
            auto message = std_msgs::msg::String();
            message.data = "hello " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger() , "publisher  : %s" , message.data.c_str());
            publisher_->publish(message);
        }
};

int main(int argc , char** argv){
    rclcpp::init(argc , argv);
    auto node = std::make_shared<TopicPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}