
#include "rclcpp/rclcpp.hpp"
#include "test02_interface/srv/zhang.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using sensor_msgs::msg::LaserScan;
using test02_interface::srv::Zhang;
using namespace std::chrono_literals;
using namespace std::placeholders;

class Test02Client : public rclcpp::Node
{
private:
    rclcpp::Client<Zhang>::SharedPtr test02_client_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr test02_subscrption_;
    float zhang_ai_90;
    float zhang_ai_180;
    float zhang_ai_270;

    void Test02Sub(const LaserScan &lei_da_pose)
    {
        float angle_90 = 1.57;
        float angle_180 = 3.14;
        float angle_270 = 4.71;
        float angle_increment = lei_da_pose.angle_increment;
        float i_90 = (angle_90 - lei_da_pose.angle_min) / angle_increment;
        float i_180 = (angle_180 - lei_da_pose.angle_min) / angle_increment;
        float i_270 = (angle_270 - lei_da_pose.angle_min) / angle_increment;
        zhang_ai_90 = lei_da_pose.ranges[i_90];
        zhang_ai_180 = lei_da_pose.ranges[i_180];
        zhang_ai_270 = lei_da_pose.ranges[i_270];

    }

public:
    float GetZhangAi90()
    {
        return zhang_ai_90;
    }

    float GetZhangAi180()
    {
        return zhang_ai_180;
    }

    float GetZhangAi270()
    {
        return zhang_ai_270;
    }

    Test02Client() : Node("minimal_client")
    {
        test02_client_ = this->create_client<Zhang>("test02");
        RCLCPP_INFO(this->get_logger(), "等待服务器连接");
        test02_subscrption_ = this->create_subscription<LaserScan>("/scan", 10,
                                        std::bind(&Test02Client::Test02Sub, this, _1));
    }

    bool connect_server()
    {
        while (test02_client_->wait_for_service(1s) == false)
        {
            if (rclcpp::ok() == false)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "强制退出");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "等待连接");
        }
        return true;
    }

    auto send(float zhang_ai_90, float zhang_ai_180, float zhang_ai_270)
    {
        auto request = std::make_shared<Zhang::Request>();
        request->num1 = zhang_ai_90;
        request->num2 = zhang_ai_180;
        request->num3 = zhang_ai_270;
        return test02_client_->async_send_request(request);
    }
};

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "请传入两个整数");
        return 1;
    }
    rclcpp::init(argc, argv);
    auto client = std::make_shared<Test02Client>();
    auto flag = client->connect_server();
    if (flag == false)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接失败");
        return 0;
    }

    Test02Client client_;
    auto respone = client->send(client_.GetZhangAi90(), client_.GetZhangAi180(), client_.GetZhangAi270());
    if (rclcpp::spin_until_future_complete(client, respone) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(client->get_logger(), "响应结果");
    }
    rclcpp::shutdown();
    return 0;
}