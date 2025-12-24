#include "rclcpp/rclcpp.hpp"
#include "base_interface_demo/msg/student.hpp"
using base_interface_demo::msg::Student;

class MinimalSubscription : public rclcpp::Node{
    private:
        rclcpp::Subscription<Student>::SharedPtr subscription_;
    public:
        MinimalSubscription() : Node("student_subscription") {
            subscription_ = this->create_subscription<Student>("topic_stu" , 10 , std::bind(&MinimalSubscription::topic_callback , this ,
                std::placeholders::_1));
        }
    private:
        void topic_callback(const Student& msg) const {
            RCLCPP_INFO(this->get_logger() , "学生: %s , %d , %.2f" , msg.name.c_str() ,msg.age ,msg.height);
            
        }
};

int main(int argc , char** argv){
    rclcpp::init(argc , argv);
    auto node = std::make_shared<MinimalSubscription>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}