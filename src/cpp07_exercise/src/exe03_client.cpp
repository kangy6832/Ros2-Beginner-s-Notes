#include "rclcpp/rclcpp.hpp"
#include "base_interface_demo/srv/distance.hpp"
#include "turtlesim/srv/spawn.hpp"
using namespace std::chrono_literals;
using base_interface_demo::srv::Distance;

//3.定义节点类
class ExeDistanceClient : public rclcpp::Node{
    private:
        rclcpp::Client<Distance>::SharedPtr distance_client;
    public:
        ExeDistanceClient() : Node("exe_distance_client"){
            //3-1.创建客户端
            distance_client = this->create_client<Distance>("distance");
        }
        //3-2.连接服务器
        bool connect_server() {
            while(!distance_client->wait_for_service(1s)){
                if(!rclcpp::ok()){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "强制退出");
                    return false ;
                }
                RCLCPP_INFO(this->get_logger(), "正在连接中");
            }
            return true;
        }
        //3-3.发送消息
        auto send_distance (float x, float y, float theta){
            auto distance_request = std::make_shared<Distance::Request>();
            distance_request->x = x;
            distance_request->y = y;
            distance_request->theta = theta;
            return distance_client->async_send_request(distance_request);
        }
};
int main(int argc, char** argv){
    if(argc!=5){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "传入(x, y, theta)");
        return 1;
    }
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ExeDistanceClient>();
    float x = std::stof(argv[1]);
    float y = std::stof(argv[2]);
    float theta = std::stof(argv[3]);
    //服务器连接
    bool flag = client->connect_server();
    if(flag == false){
        RCLCPP_INFO(client->get_logger(), "连接失败");
        return 0;
    }
    //发送请求
    auto distance_future = client->send_distance(x, y, theta);
    //处理响应
    if(rclcpp::spin_until_future_complete(client, distance_future) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(client->get_logger(), "两只乌龟相距：%.2f米", distance_future.get()->distance);
    } else {
        RCLCPP_INFO(client->get_logger(), "获取距离服务失败");
    }
    rclcpp::shutdown();
    return 0;
}

