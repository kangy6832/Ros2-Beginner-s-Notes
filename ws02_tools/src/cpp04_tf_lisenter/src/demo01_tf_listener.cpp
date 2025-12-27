/*  
  需求：订阅 laser 到 base_link 以及 camera 到 base_link 的坐标系关系，
       并生成 laser 到 camera 的坐标变换。
  步骤：
    1.包含头文件；
    2.初始化 ROS 客户端；
    3.定义节点类；
      3-1.创建tf缓存对象指针,融合多个坐标系关系为一颗坐标树；
      3-2.创建tf监听器，将所有广播的数据写入缓存；
      3-3.按照条件查找符合条件的坐标系并生成变换后的坐标帧，编写定时器，循环实现转换
    4.调用 spin 函数，并传入对象指针；
    5.释放资源。

*/
/*

*/
#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
using namespace std::chrono_literals;

class TFLisenter : public rclcpp::Node {
    public:
        TFLisenter():Node("tf_lisenter_node_cpp"){
            //3.1创建缓存对象
            buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock()); 
            //3.2创建监听器对象
            lisenter_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, this);
            //3.3编写定时器 
            timer_ = this->create_wall_timer(100ms, std::bind(&TFLisenter::on_timer, this));
        }
    private:
        void on_timer(){
            //实现坐标系转换
            try{
            //目标坐标系  源坐标系  （雷达能检测到障碍物相对于自身的位置）
            //TimePointZero:获取最新可用的坐标变换
            //geometry_msgs::msg::TransformStamped 返回新的坐标帧
            //lookupTransform(const std::string &target_frame,  //新坐标帧的父坐标系
            //const std::string &source_frame,  //新坐标帧的子坐标系
            //const tf2::TimePoint &time)  //转换的时间点，tf2::TimePointZero转换最新时刻的坐标帧
            //当转换失败时会抛出异常，用try catch处理
                auto transformStamped = buffer_->lookupTransform("camera", "laser", tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(),"----------------------转换结果----------------------");
                RCLCPP_INFO(this->get_logger(),"frame_id:%s", transformStamped.header.frame_id.c_str());
                RCLCPP_INFO(this->get_logger(),"child_frame_id:%s",transformStamped.child_frame_id.c_str());
                RCLCPP_INFO(this->get_logger(),"坐标:(%.2f,%.2f,%.2f)",
                    transformStamped.transform.translation.x,
                    transformStamped.transform.translation.y,
                    transformStamped.transform.translation.z);
            }
            catch (const tf2::LookupException& e){
                RCLCPP_INFO(this->get_logger(), "异常，%s", e.what());
            }
        }
        std::unique_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> lisenter_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFLisenter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

