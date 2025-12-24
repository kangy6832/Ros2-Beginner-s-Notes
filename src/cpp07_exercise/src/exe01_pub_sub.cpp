#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp> //x,y坐标。theta(朝向角度)
#include <geometry_msgs/msg/twist.hpp> //线速度和角速度
class ExePubSub : public rclcpp::Node {
  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  public:
    ExePubSub():Node("exe01_pub_sub"){
      twist_pub_=this->create_publisher<geometry_msgs::msg::Twist>("/t2/turtle1/cmd_vel", 1 );
      pose_sub_=this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 1, 
        std::bind(&ExePubSub::posecallback, this, std::placeholders::_1));
    }
  private:
    void posecallback(const turtlesim::msg::Pose::ConstSharedPtr pose){
      geometry_msgs::msg::Twist twist;
      twist.angular.z = -(pose->angular_velocity);
      twist.linear.x = pose->linear_velocity;
      twist_pub_->publish(twist);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExePubSub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
