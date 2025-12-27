#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals;

class MinimalParamClient : public rclcpp::Node{
    public:
        MinimalParamClient():Node("paramDemoClient_node"){
            parameter_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "minimal_param_server");
        }
        bool connect_server(){
            while(parameter_client_->wait_for_service(10s) == false){
                if(rclcpp::ok() == false ){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "");
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "服务器未连接");
            }
            return true;
        }

        void get_param(){
            auto height = parameter_client_->get_parameter<double>("  ");
            auto paramers = parameter_client_->get_parameters({"  ", "  ", "  "});
            for(auto param : paramers){
                RCLCPP_INFO(this->get_logger(), "%s = %s", param.get_name().c_str(), param.value_to_string().c_str());
            }
        }

        void update_param(){
            parameter_client_->set_parameters({rclcpp::Parameter("", ), rclcpp::Parameter("", "")});
        }
    private:
        rclcpp::SyncParametersClient::SharedPtr parameter_client_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto param = std::make_shared<MinimalParamClient>();
    bool flag = param->connect_server();
    if(!flag){
        return 0;
    }    
    param->get_param();
    rclcpp::shutdown();
    return 0;
}