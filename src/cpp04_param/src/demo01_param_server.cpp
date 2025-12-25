/*
${workspaceFolder}/**
/opt/ros/humble/include/**
/opt/ros/humble/include/std_msgs/**
/opt/ros/humble/include/rclcpp/**
/opt/ros/humble/include/rcl/**
/opt/ros/humble/include/rclcpp_action/**
/opt/ros/humble/include/rclcpp
*/

/*
declare_parameter         初始化参数，设置默认值
set_parameter             改变参数的值
get_parameter             读取参数当前的值
has_parameter             判断参数是否已声明/存在
param.get_name()          获取Parameter对象的参数名
param.value_to_string     将任意类型的参数值转为字符串
undeclare_parameter       移除已声明的参数
*/
#include "rclcpp/rclcpp.hpp"
class MinimalParamServer : public rclcpp::Node{
    public:
        // 允许节点接收和处理未声明的参数
        MinimalParamServer():Node("minimal_param_server",rclcpp::NodeOptions().allow_undeclared_parameters(true)){}
        //声明函数
        void declare_param(){
            this->declare_parameter("car_type","Tiger");
            this->declare_parameter("height",1.50);
            this->declare_parameter("wheels",4);
            this->set_parameter(rclcpp::Parameter("undcl_test",100));
        }
        //查询函数
        void get_param(){
            RCLCPP_INFO(this->get_logger(),"------------查-----------------------");
            //获取制定
            rclcpp::Parameter car_type = this->get_parameter("car_type");
            RCLCPP_INFO(this->get_logger(), "car_type: %s", car_type.as_string().c_str());
            RCLCPP_INFO(this->get_logger(),"height:%.2f",this->get_parameter("height").as_double());
            RCLCPP_INFO(this->get_logger(),"wheels:%ld",this->get_parameter("wheels").as_int());
            RCLCPP_INFO(this->get_logger(),"undcl_test:%ld",this->get_parameter("undcl_test").as_int());
            //判断包含
            RCLCPP_INFO(this->get_logger(),"包含car_type?%d",this->has_parameter("car_type"));
            RCLCPP_INFO(this->get_logger(),"包含car_typexxxx?%d",this->has_parameter("car_typexxxx"));
            //获取所有
            // std::vector<rclcpp::Parameter> 容器
            auto params = this->get_parameters({"car_type","height","wheels"});
            for(auto &param : params){
                RCLCPP_INFO(this->get_logger(),"name = %s , value = %s" ,param.get_name().c_str() , param.value_to_string().c_str());
            }
        }
        //修改参数
        void update_param(){
            RCLCPP_INFO(this->get_logger(),"------------------改----------------");
            this->set_parameter(rclcpp::Parameter("height",1.75));
            RCLCPP_INFO(this->get_logger(),"height%.2f",this->get_parameter("height").as_double());
        }
        //删除参数
        void del_param(){
            RCLCPP_INFO(this->get_logger(),"------------------删----------------");
            this->undeclare_parameter("car_type");
            RCLCPP_INFO(this->get_logger(),"删除操作后，car_type 还存在马? %d",this->has_parameter("car_type"));
            RCLCPP_INFO(this->get_logger(),"删除操作前，undcl_test存在吗？%d",this->has_parameter("undcl_test"));
            this->undeclare_parameter("undcl_test");
            RCLCPP_INFO(this->get_logger(),"删除操作前，undcl_test 存在吗? %d",this->has_parameter("undcl_test"));
        }
};
int main(int argc , char** argv){
    rclcpp::init(argc ,argv);
    auto paramServer = std::make_shared<MinimalParamServer>();
    paramServer->declare_param();
    paramServer->get_param();
    paramServer->update_param();
    paramServer->del_param();
    rclcpp::spin(paramServer);
    rclcpp::shutdown();
    return 0;
}