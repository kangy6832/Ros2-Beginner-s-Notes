/*
   需求：向动作服务端发送目标点数据，并处理服务端的响应数据。
   步骤：
       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
            3-1.创建动作客户端；
            3-2.发送请求数据，并处理服务端响应；
            3-3.处理目标响应；
            3-4.处理响应的连续反馈；
            3-5.处理最终响应。
       4.调用spin函数，并传入节点对象指针；
       5.释放资源。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/nav.hpp"
#include "turtlesim/srv/spawn.hpp"

using base_interfaces_demo::action::Nav;
using namespace std::chrono_literals;
using namespace std::placeholders;

// 3.定义节点类；
class ExeNavActionClient: public rclcpp::Node{
public:
    ExeNavActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    :Node("exe_nav_action_client",options){
        // 3-1.创建动作客户端；
        nav_client = rclcpp_action::create_client<Nav>(this,"nav");
    }
    // 3-2.发送请求数据，并处理服务端响应；
    void send_goal(float x, float y, float theta){
        // 连接动作服务端，如果超时（5s），那么直接退出。
        if (!nav_client->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(this->get_logger(),"服务连接失败!");
            return;
        }
        // 组织请求数据
        auto goal_msg = Nav::Goal();
        goal_msg.goal_x = x;
        goal_msg.goal_y = y;
        goal_msg.goal_theta = theta;
        //const rclcpp_action::Client<base_interfaces_demo::action::Nav>::SendGoalOptions &options
        rclcpp_action::Client<Nav>::SendGoalOptions options;
        options.goal_response_callback = std::bind(&ExeNavActionClient::goal_response_callback, this, _1);
        options.feedback_callback = std::bind(&ExeNavActionClient::feedback_callback, this, _1, _2);
        options.result_callback = std::bind(&ExeNavActionClient::result_callback, this, _1);
        // 发送
        nav_client->async_send_goal(goal_msg,options);

    }
private:
    rclcpp_action::Client<Nav>::SharedPtr nav_client;
    // 3-3.处理目标响应；
    void goal_response_callback(rclcpp_action::ClientGoalHandle<Nav>::SharedPtr goal_handle){
        if(!goal_handle){
            RCLCPP_ERROR(this->get_logger(),"目标请求被服务器拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(),"目标请求被接收!");
        }
    }
    // 3-4.处理响应的连续反馈；
    void feedback_callback(rclcpp_action::ClientGoalHandle<Nav>::SharedPtr goal_handle, 
        const std::shared_ptr<const Nav::Feedback> feedback){
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(),"距离目标点还有 %.2f 米。",feedback->distance);
    }
    // 3-5.处理最终响应。
    void result_callback(const rclcpp_action::ClientGoalHandle<Nav>::WrappedResult & result){
        switch (result.code){
        case rclcpp_action::ResultCode::SUCCEEDED :
            RCLCPP_INFO(this->get_logger(),
                "乌龟最终坐标:(%.2f,%.2f),航向:%.2f",
                            result.result->turtle_x,
                            result.result->turtle_y,
                            result.result->turtle_theta
                            );
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(),"任务被取消");
            break;      
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(),"任务被中止");
            break;   
        default:
            RCLCPP_ERROR(this->get_logger(),"未知异常");
            break;
        }
        // rclcpp::shutdown();
    }
};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc,argv);
    // 4.调用spin函数，并传入节点对象指针；
    auto client = std::make_shared<ExeNavActionClient>();

    if (argc != 5)
    {
        RCLCPP_INFO(client->get_logger(),"请传入目标的位姿参数:(x,y,theta)");
        return 1;
    }
    // 发送目标点
    client->send_goal(atof(argv[1]), atof(argv[2]), atof(argv[3]));
    rclcpp::spin(client);
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}