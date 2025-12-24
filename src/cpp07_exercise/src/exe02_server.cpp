#include "rclcpp/rclcpp.hpp"
#include "base_interface_demo/srv/distance.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
using namespace std::chrono_literals;
class ExeDistanceServer : public rclcpp::Node{
    private:
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub;
        rclcpp::Service<base_interface_demo::srv::Distance>::SharedPtr distance_server;
        float turtle1_x;
        float turtle1_y;
    public:
        ExeDistanceServer():Node("exe_distance_server"), turtle1_x(0.0), turtle1_y(0.0){
            //3-1.创建服务端
            distance_server = this->create_service<base_interface_demo::srv::Distance>("distance", 
                std::bind(&ExeDistanceServer::distanceCallBack, this, std::placeholders::_1, std::placeholders::_2));
            //3-2.创建乌龟姿态订阅方，回调函数中获取x和y坐标;
            pose_sub = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10,
                std::bind(&ExeDistanceServer::poseCallBack, this, std::placeholders::_1));
        }
    private:
        void poseCallBack(const turtlesim::msg::Pose::SharedPtr pose){
            turtle1_x = pose->x;
            turtle1_y = pose->y;
        }
        //3-3.解析目标值，计算距离并反馈结果。
        void distanceCallBack(const base_interface_demo::srv::Distance_Request::SharedPtr request, 
                            base_interface_demo::srv::Distance_Response::SharedPtr respone){
            //解析目标值
            float goal_x = request->x;
            float goal_y = request->y;
            //计算距离
            float x = goal_x - turtle1_x;
            float y = goal_y - turtle1_y;
            //将结果设置到响应
            respone->distance = std::sqrt(x*x + y*y);
            RCLCPP_INFO(this->get_logger(), "目标坐标:(%.2f, %.2f), 距离：%.2f", goal_x, goal_y, respone->distance);
        }
};
int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExeDistanceServer>());
    rclcpp::shutdown();
    return 0;
}