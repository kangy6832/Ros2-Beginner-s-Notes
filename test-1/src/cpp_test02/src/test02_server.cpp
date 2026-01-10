#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "test02_interface/srv/zhang.hpp"

using sensor_msgs::msg::LaserScan;
using test02_interface::srv::Zhang;

class Test02Service : public rclcpp::Node{
    private:
        rclcpp::Service<Zhang>::SharedPtr test02_server_;
        bool ok_90;
        bool ok_180;
        bool ok_270;

    public:
        Test02Service() : Node("test02_service"){
            test02_server_ = this->create_service<Zhang>("test02",std::bind(&Test02Service::Test02Ser, this ,std::placeholders::_1 , std::placeholders::_2));
        }
    private:
        void Test02Ser (const Zhang::Request::SharedPtr req , const Zhang::Response::SharedPtr res)  {
            if(req->num1 > 1){
                ok_90 = false;
                RCLCPP_INFO(this->get_logger(), "90度方向有障碍物请调转方向");
            }
            if(req->num2 > 1){
                ok_180 = false;
                RCLCPP_INFO(this->get_logger(), "180度方向有障碍物请调转方向");
            }
            if(req->num3 > 1){
                ok_270 = false;
                RCLCPP_INFO(this->get_logger(), "270度方向有障碍物请调转方向");
            }
            res->ok_num1 = ok_90;
            res->ok_num2 = ok_180;
            res->ok_num3 = ok_270;

            
        }
};

int main(int argc , char** argv){
    rclcpp::init(argc , argv);
    auto node = std::make_shared<Test02Service>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}