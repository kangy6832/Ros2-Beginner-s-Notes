#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"  //string 消息的接口
using namespace std::chrono_literals;  //能使用时间单位 如： 100ms

class MinimalPublisher :  public rclcpp::Node{
    private:
      rclcpp::TimerBase::SharedPtr timer_;  //创建时间变量，实现定时动作。
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  //创建发布者对象
      size_t count_;  //记录次数
    public:
        MinimalPublisher() : Node("minimal_publisher"), count_(0){
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic" , 10);  //创建std_msgs::msg::String类型的发布者，话题名称“topic”，数据深度10
            timer_ = this->create_wall_timer(1000ms , std::bind(&MinimalPublisher::timer_callback,this));  //create_wall_timer()创建时间间隔为1000ms ， 执行封装函数 ， **不要忘加this**
        }
    private:
        void timer_callback(){
            auto message = std_msgs::msg::String();  //调用类的构造函数
            message.data = "Hello world!!! " + std::to_string(count_++);  //data为类内存储数据的对象
            RCLCPP_INFO(this->get_logger() , "发布的消息：%s",message.data.c_str());  //用输出宏将get_logger() 获得的信息输出，因为以c风格输出，故要将string类型用c_str()转到c语言的字符串风格
            publisher_->publish(message);  //调用publish成员函数发布消息
        }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc , argv);
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  printf("hello world demo01_talker_cpp_str package\n");
  return 0;
}
