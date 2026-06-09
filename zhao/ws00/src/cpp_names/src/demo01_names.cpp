#include "rclcpp/rclcpp.hpp"
    
    
class publish:public rclcpp::Node
{
public:
    publish():Node("publish_node_name")
    {
        //创建节点
        RCLCPP_INFO(this->get_logger(),"publish节点创建！");
        
    }
    
private:
    
};
    
    
int main(int argc, char ** argv)
{
    // 初始化
    rclcpp::init(argc,argv);
        
    // 调用spin函数
    rclcpp::spin(std::make_shared<publish>());
    
    // 释放资源
    rclcpp::shutdown();
    
    return 0;
}