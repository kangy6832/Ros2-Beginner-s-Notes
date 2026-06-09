/*  
  需求：订阅 laser 到 base_link 以及 camera 到 base_link 的坐标系关系，
       并生成 laser 到 camera 的坐标变换。
  步骤：
    1.包含头文件；
    2.初始化 ROS 客户端；
    3.定义节点类；
      3-1.创建tf缓存对象指针；
      3-2.创建tf监听器；
      3-3.按照条件查找符合条件的坐标系并生成变换后的坐标帧。
    4.调用 spin 函数，并传入对象指针；
    5.释放资源。

*/
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

// 3.定义节点类；
class MinimalFrameListener : public rclcpp::Node {
public:
  MinimalFrameListener():Node("minimal_frame_listener"){
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = this->create_wall_timer(1s, std::bind(&MinimalFrameListener::on_timer,this));
  }

private:
  void on_timer(){
    try
    {
      auto transformStamped = tf_buffer_->lookupTransform("camera","laser",tf2::TimePointZero);
      RCLCPP_INFO(this->get_logger(),"----------------------转换结果----------------------");
      RCLCPP_INFO(this->get_logger(),"frame_id:%s",transformStamped.header.frame_id.c_str());
      RCLCPP_INFO(this->get_logger(),"child_frame_id:%s",transformStamped.child_frame_id.c_str());
      RCLCPP_INFO(this->get_logger(),"坐标:(%.2f,%.2f,%.2f)",
                transformStamped.transform.translation.x,
                transformStamped.transform.translation.y,
                transformStamped.transform.translation.z);

    }
    catch(const tf2::LookupException& e)
    {
      RCLCPP_INFO(this->get_logger(),"坐标变换异常：%s",e.what());
    }


  }
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char const *argv[])
{
  // 2.初始化 ROS 客户端；
  rclcpp::init(argc,argv);
  // 4.调用 spin 函数，并传入对象指针；
  rclcpp::spin(std::make_shared<MinimalFrameListener>());
  // 5.释放资源。
  rclcpp::shutdown();
  return 0;
}