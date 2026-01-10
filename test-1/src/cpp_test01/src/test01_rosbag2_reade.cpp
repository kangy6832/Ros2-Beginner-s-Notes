#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h> 
#include <tf2_ros/transform_broadcaster.h> 
#include <chrono>
using geometry_msgs::msg::Twist;
using namespace std::placeholders;
using namespace std::chrono_literals;

class Test01 : public rclcpp::Node {
    public:
        Test01() : Node("Test01_rosbag"){

            time_ = this->now();

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            test01_subscription_ = this->create_subscription<Twist>("/cmd_vel", 
                10, std::bind(&Test01::sub_, this, _1));
            
                timer_ = this->create_wall_timer(1000ms, std::bind(&Test01::handle_twist, this));

        }
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<Twist>::SharedPtr test01_subscription_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        float x_translation;
        float y_translation;
        float z_translation;
        float angular_x_ ;
        float angular_y_ ;
        float angular_z_ ;
        rclcpp::Time time_;
        

        void sub_ (const geometry_msgs::msg::Twist::SharedPtr msg){
            auto begin_time_ = this->now();
            float _time_ = (begin_time_ - time_).seconds();
            x_translation += (msg->linear.x) * _time_;
            y_translation += (msg->linear.y) * _time_;
            z_translation += (msg->linear.z) * _time_;
            angular_x_ += msg->angular.x * _time_;
            angular_y_ += msg->angular.y * _time_;
            angular_z_ += msg->angular.z * _time_;
        }

        void handle_twist(){
            geometry_msgs::msg::TransformStamped t;
            rclcpp::Time now = this->get_clock()->now();
            t.header.stamp = now;
            t.header.frame_id = "World";
            t.child_frame_id = "Laser";
            t.transform.translation.x = x_translation;
            t.transform.translation.y = y_translation;
            t.transform.translation.z = z_translation;
            tf2::Quaternion q;
            q.setRPY(angular_x_, angular_y_, angular_z_);
            t.transform.rotation.x = q.x() ;
            t.transform.rotation.y = q.y() ;
            t.transform.rotation.z = q.z() ;
            t.transform.rotation.w = q.w() ;
            tf_broadcaster_ -> sendTransform(t);

        }
};
int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node01 = std::make_shared<Test01>();
    rclcpp::spin(node01);
    rclcpp::shutdown();
    return 0;
}