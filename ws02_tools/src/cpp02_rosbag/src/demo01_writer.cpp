
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
/opt/ros/humble/include/rosbag2_cpp 
*/

//结构化：使数据有固定组织规则
//原生消息：如geometry_msgs::msg::Twist是结构化的c++字段
//序列化：将结构化的原生消息转化成二进制字节流
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "geometry_msgs/msg/twist.hpp"  //立体线速度和角速度
using std::placeholders::_1;

class SimpleBagRecorder : public rclcpp::Node{
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
        std::unique_ptr<rosbag2_cpp::Writer> writer_; //创建独占式智能指针
    public:
        SimpleBagRecorder() : Node("simple_bag_recorder"){
            writer_ = std::make_unique<rosbag2_cpp::Writer>();
            writer_->open("my_bag"); //rosbag2写入器初始化并创建（或打开）my_bag文件夹
            subscription_ = create_subscription<geometry_msgs::msg::Twist>(
                "/turtle1/cmd_vel",10 , std::bind(&SimpleBagRecorder::topic_callback , this , _1));
        }
    private:
        void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const { 
            //rclcpp::SerializedMessage：存储序列化数据的容器
            rclcpp::Time time_stamp = this->now(); //为写入 rosbag 的每条 /turtle1/cmd_vel 消息打上时间标记
            writer_->write(msg, "/turtle1/cmd_vel", "geometry_msgs/msg/Twist", time_stamp); //保证回放时按时间顺序播放
        }
};

int main(int argc , char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleBagRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
