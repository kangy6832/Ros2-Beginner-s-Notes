#include "rclcpp/rclcpp.hpp"
#include "base_interface_demo/msg/student.hpp"

using namespace std::chrono_literals;
using base_interface_demo::msg::Student;
class MinimaPubilsher : public rclcpp::Node{
    private:
        rclcpp::Publisher<Student>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;
    public:
        MinimaPubilsher():Node("student_publisher"){
            publisher_ = this->create_publisher<Student>("topic_stu" , 10);
            timer_ = this->create_wall_timer(100ms , std::bind(&MinimaPubilsher::timer_callback , this));
        }
    private:
        void timer_callback(){
            auto stu = Student();
            stu.name = "张三";
            stu.age = 19 ;
            stu.height = 348;
            RCLCPP_INFO(this->get_logger(),"学生%s , %d , %.2f",stu.name.c_str(), stu.age ,stu.height);
            publisher_->publish(stu);
        }
};

int main(int argc , char** argv){
    rclcpp::init(argc , argv);
    auto node = std::make_shared<MinimaPubilsher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


