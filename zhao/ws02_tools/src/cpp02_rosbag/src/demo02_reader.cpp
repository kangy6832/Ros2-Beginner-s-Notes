#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SimpleBagPlayer : public rclcpp::Node{
    private:
        std::unique_ptr<rosbag2_cpp::Reader> reader_;
    public:
        SimpleBagPlayer():Node("simple_bag_player"){
            //创建读取对象
            reader_ = std::make_unique<rosbag2_cpp::Reader>();
            //打开文件
            reader_->open("my_bag");
            //读
            while(reader_->has_next()){
                geometry_msgs::msg::Twist twist = reader_->read_next<geometry_msgs::msg::Twist>();
                //read_next：读取下一条消息，读取指针最初在第一条消息之前
                RCLCPP_INFO(this->get_logger() ,"%.2f ---- %.2f", twist.linear.x, twist.angular.z);
            }
            //关闭文件
            reader_->close(); //关闭，不关闭会占用读句柄，导致写句柄不能使用
        }
};
int main(int argc , char** argv){
    rclcpp::init(argc , argv);
    auto node = std::make_shared<SimpleBagPlayer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}