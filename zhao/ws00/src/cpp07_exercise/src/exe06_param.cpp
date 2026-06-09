/*
   需求：修改turtlesim_node的背景颜色。
   步骤：
       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
            3-1.创建参数客户端；
            3-2.连接参数服务端；
            3-3.更新参数。
       4.创建对象指针,并调用其函数；
       5.释放资源。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
// 3.定义节点类；
class ExeParamClient: public rclcpp::Node{
public:
    ExeParamClient():Node("exe_param_client"),red(0){
        // 3-1.创建参数客户端；
        param_client = std::make_shared<rclcpp::SyncParametersClient>(this,"/turtlesim");
    }
    // 3-2.连接参数服务端；
    bool connect_server(){
        while (!param_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(),"终端退出!");
                return false;
            }

            RCLCPP_INFO(this->get_logger(),"参数服务连接中，请稍等......");
        }
        return true;
    }
    // 3-3.更新参数。
    void update_param(){
        red = param_client->get_parameter<int32_t>("background_r");
        rclcpp::Rate rate(30.0);
        int i = red;
        while (rclcpp::ok())
        {
            i < 255 ? red += 5 : red -= 5;
            i += 5;
            if(i >= 510) i = 0;

            // RCLCPP_INFO(this->get_logger(),"red = %d", red);
            param_client->set_parameters({rclcpp::Parameter("background_r",red)});
            rate.sleep();
        }

    }
private:
    rclcpp::SyncParametersClient::SharedPtr param_client;
    int32_t red;
};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc,argv);

    // 4.创建对象指针,并调用其函数；
    auto param_client = std::make_shared<ExeParamClient>();
    if(!param_client->connect_server()) return 1;
    param_client->update_param();

    // rclcpp::spin(param_client);
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}