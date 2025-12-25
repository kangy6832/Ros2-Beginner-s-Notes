/*
${workspaceFolder}/**
/opt/ros/humble/include/**
/opt/ros/humble/include/std_msgs/**
/opt/ros/humble/include/rclcpp/**
/opt/ros/humble/include/rcl/**
/opt/ros/humble/include/rclcpp_action/**
/opt/ros/humble/include/rclcpp
/opt/ros/humble/include/rcl
*/

#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;
class MinimalParamClient : public rclcpp::Node {
    private:
        rclcpp::SyncParametersClient::SharedPtr paramClient;
    public:
        MinimalParamClient():Node("paramDemoClient_node"){
            paramClient = std::make_shared<rclcpp::SyncParametersClient>(this , "minimal_param_server");
        }
        bool connect_server(){
            //等待服务器连接
            while (!paramClient->wait_for_service(1s)){
                if(!rclcpp::ok()){
                    return false;
                }
                RCLCPP_INFO(this->get_logger(),"服务器未连接");
            }
            return true;
        }
        //查询参数
        void get_param(){
            RCLCPP_INFO(this->get_logger(),"-----------参数客户端查询参数-----------");
            // get_parameter - 获取参数的基本函数  <double> - 模板参数，指定期望的返回类型  ("height") - 要获取的参数名称
            double height = paramClient->get_parameter<double>("height");
            RCLCPP_INFO(this->get_logger(),"height = %.2f",height);
            RCLCPP_INFO(this->get_logger(),"car_type存在吗? %d" , paramClient->has_parameter("car_type"));
            auto params = paramClient->get_parameters({"car_type","height","wheels"});
            for(auto& param : params){
                RCLCPP_INFO(this->get_logger(),"%s = %s",param.get_name().c_str(),param.value_to_string().c_str());
            }
        }
        //修改函数
        void update_param(){
            RCLCPP_INFO(this->get_logger(),"-----------参数客户端修改参数-----------");
            paramClient->set_parameters({rclcpp::Parameter("car_type","Mouse"),rclcpp::Parameter("height",2.0),
            rclcpp::Parameter("width",0.15),
            rclcpp::Parameter("wheels",6)});
        }
};

int main(int argc , char** argv){
    rclcpp::init(argc , argv);
    auto paramClient = std::make_shared<MinimalParamClient>();
    bool flag = paramClient->connect_server();
    if(!flag){
        return 0;
    }
    paramClient->get_param();
    paramClient->update_param();
    paramClient->get_param();
    rclcpp::shutdown();
    return 0;
}