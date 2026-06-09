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
${workspaceFolder}/chapt1/ws01/build/interface_demo/rosidl_generator_cpp
 */


#include <rclcpp/rclcpp.hpp>
#include <interface_demo/srv/addint.hpp>
using interface_demo::srv::Addint;
using namespace std::placeholders;

class MinimalService : public rclcpp::Node {
    public:
        MinimalService():Node("minimal_service"){
            service_ = this->create_service<Addint>("get_sum", std::bind(&MinimalService::add, this, _1, _2));
        }
    private:    
        rclcpp::Service<Addint>::SharedPtr service_;
        void add(const Addint::Request::SharedPtr req_ , const Addint::Response::SharedPtr res_){
            res_->sum = req_->num1 + req_->num2;
            RCLCPP_INFO(this->get_logger(), "request:%d, %d respone:%d", req_->num1, req_->num2, res_->sum);
        }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}