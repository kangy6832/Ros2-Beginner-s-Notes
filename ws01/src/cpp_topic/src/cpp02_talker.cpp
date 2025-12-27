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
 */

#include "rclcpp/rclcpp.hpp"
#include "interface_demo/msg/student.hpp"
using namespace std::chrono_literals;
using interface_demo::msg::Student;

class TopicPublisherStudent : public rclcpp::Node{
    private:
        rclcpp::Publisher<Student>::SharedPtr stu_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

    public:
        TopicPublisherStudent() : Node("cpp02_talker") , count_(0){
            stu_publisher_ = this->create_publisher<Student>("topic_02" , 10);
            timer_ = this->create_wall_timer(1000ms , std::bind(&TopicPublisherStudent::CoutStudent , this));
        }

    private:
        void CoutStudent () {
            auto message = Student();
            message.age = 18;
            message.height = 1.89;
            message.name = "小明";
            RCLCPP_INFO(this->get_logger() , "publisher: %s , %d, %.2f",message.name.c_str(),message.age, message.height);
            stu_publisher_->publish(message);
        }
};

int main(int argc , char** argv){
    rclcpp::init(argc , argv);
    auto node = std::make_shared<TopicPublisherStudent>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
