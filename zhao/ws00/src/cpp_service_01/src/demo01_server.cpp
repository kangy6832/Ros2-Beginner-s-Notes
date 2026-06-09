#include "rclcpp/rclcpp.hpp"
#include "base_interface_demo/srv/add_int.hpp"
using base_interface_demo::srv::AddInt;

class MinimalService : public rclcpp::Node{
    private:
        rclcpp::Service<AddInt>::SharedPtr server_;
    public:
        MinimalService() : Node("minimal_service"){
            server_ = this->create_service<AddInt>("add_int",std::bind(&MinimalService::add , this ,std::placeholders::_1 , std::placeholders::_2));
        }
    private:
        void add(const AddInt::Request::SharedPtr req , const AddInt::Response::SharedPtr res)  {
            //const可以修饰指针本身，指针绑定，不能再指向其他数据。
            res->sum = req->num1 + req->num2;
            RCLCPP_INFO(get_logger(),"请求：%d %d ,响应 %d",req->num1 , req->num2 ,res->sum);
        }
};

int main(int argc , char** argv){
    rclcpp::init(argc , argv);
    auto node = std::make_shared<MinimalService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

