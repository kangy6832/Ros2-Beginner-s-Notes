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
/opt/ros/humble/include/geometry_msgs */
#include <geometry_msgs/msg/transform_stamped.hpp> //创建坐标变换消息
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h> //欧拉角向四元数转换
#include <tf2_ros/static_transform_broadcaster.h> //静态变换广播器
using std::placeholders::_1;
class MinimalStaticFrameBroadcaster : public rclcpp::Node {
    private:
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
        //专门广播静态变换，
    public:
        explicit MinimalStaticFrameBroadcaster(char* transformation[]):Node("minimal_static_frame_broadcaster"){
            tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            this->make_transforms(transformation); //广播
        }
    private:
        void make_transforms(char* transformation[]){
            //组织消息
            geometry_msgs::msg::TransformStamped t;
            rclcpp::Time now = this->get_clock()->now();
            t.header.stamp = now;
            t.header.frame_id = transformation[7];
            t.child_frame_id = transformation[8];
            t.transform.translation.x = atof(transformation[1]);
            t.transform.translation.y = atof(transformation[2]);
            t.transform.translation.z = atof(transformation[3]);
            tf2::Quaternion q;
            q.setRPY(
                atof(transformation[4]),
                atof(transformation[5]), 
                atof(transformation[6])   //欧拉角转换成四元数
            ); 
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            //发布消息
            tf_publisher_->sendTransform(t);
        }
};

int main(int argc, char** argv){
    //判断终端传入的参数是否合法
    auto logger = rclcpp::get_logger("logger");
    if(argc != 9){
        RCLCPP_INFO(logger, "运行程序时请按照：x y z roll pitch yaw frame_id child_frame_id 的格式传入参数");
        return 1;
    }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalStaticFrameBroadcaster>(argv));
    rclcpp::shutdown();
    return 0;
}