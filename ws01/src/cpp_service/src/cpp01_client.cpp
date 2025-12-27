#include <rclcpp/rclcpp.hpp>
#include <interface_demo/srv/addint.hpp>
using interface_demo::srv::Addint;
using namespace std::placeholders;
using namespace std::chrono_literals;

class CppClient : public rclcpp::Node{
    public:
        CppClient() : Node("minimal_client"){
            client_ = this->create_client<Addint>("add_int");
            RCLCPP_INFO(this->get_logger(), "等待服务器连接");
        }

        bool service_connect(){
            while(!client_->wait_for_service(1s)){
                if(!rclcpp::ok()){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "强制退出");
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "等待连接");
            }
            return true;
        }

        auto send_resquest (const int32_t num1, int32_t num2){
            auto request = std::make_shared<Addint::Request>();
            request->num1 = num1;
            request->num2 = num2;
            RCLCPP_INFO(get_logger(), "发送请求：%d %d", num1, num2);
            return client_->async_send_request(request);
        }
    private:
        rclcpp::Client<Addint>::SharedPtr client_;


};

int main(int argc , char** argv){
    if(argc != 3){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请传入两个整数");
        return 0;
    }
    rclcpp::init(argc, argv);
    auto client = std::make_shared<CppClient>();
    auto flag = client->service_connect();
    if(flag == false){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "服务器连接失败");
        return 0;
    }
    auto respone = client->send_resquest(std::stoi(argv[1]), std::stoi(argv[2]));
    if(rclcpp::spin_until_future_complete(client, respone) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "响应结果：%d", respone.get()->sum);
    }
    rclcpp::shutdown();
    return 0;
}