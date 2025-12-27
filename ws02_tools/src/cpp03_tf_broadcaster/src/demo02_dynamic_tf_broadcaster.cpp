/* ${workspaceFolder}/**
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
${workspaceFolder}/chapt1/ws01/build/interface_demo/rosidl_generator_cpp
/opt/ros/humble/include/rosidl_runtime_cpp
/opt/ros/humble/include/geometry_msgs
/opt/ros/humble/include/turtlesim */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/msg/pose.hpp>
using std::placeholders::_1;
class MinimalDynamicFrameBroadcaster : public rclcpp::Node{
    private:
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    public:
        MinimalDynamicFrameBroadcaster():Node("minimal_dynamic_frame_broadcaster"){
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            std::string topic_name = "/turtle1/pose";
            subscription_ = this->create_subscription<turtlesim::msg::Pose>(
                topic_name, 10, std::bind(&MinimalDynamicFrameBroadcaster::handle_turtle_pose, this, _1)
            );
        }
    private:
        void handle_turtle_pose(const turtlesim::msg::Pose& msg){
            geometry_msgs::msg::TransformStamped t;
            rclcpp::Time now = this->get_clock()->now();
            t.header.stamp = now;
            t.header.frame_id = "world";
            t.child_frame_id = "turtle1";
            t.transform.translation.x = msg.x;
            t.transform.translation.y = msg.y;
            t.transform.translation.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, msg.theta);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(t);
        }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalDynamicFrameBroadcaster>());
    rclcpp::shutdown();
    return 0 ;
}