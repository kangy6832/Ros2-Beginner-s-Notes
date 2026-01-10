// ROS2 Action Server 模板
// 注意：需要替换以下占位符：
// 1. base_interfaces_demo/action/progress.hpp -> 您的action头文件
// 2. Progress -> 您的action类型
// 3. MinimalActionServer -> 您的类名
// 4. minimal_action_server -> 您的节点名
// 5. get_sum -> 您的action名称

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"
using namespace std::placeholders;
using base_interfaces_demo::action::Progress;
using GoalHandleProgress = rclcpp_action::ServerGoalHandle<Progress>;

class MinimalActionServer : public rclcpp::Node {
private:
    rclcpp_action::Server<Progress>::SharedPtr action_server_;

public:
    explicit MinimalActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("minimal_action_server", options) {
        this->action_server_ = rclcpp_action::create_server<Progress>(
            this,
            "get_sum",
            std::bind(&MinimalActionServer::handle_goal, this, _1, _2),
            std::bind(&MinimalActionServer::handle_cancel, this, _1),
            std::bind(&MinimalActionServer::handle_accepted, this, _1));
        RCLCPP_INFO(this->get_logger(), "动作服务端创建，等待请求！");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const Progress::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "接收到动作客户端请求，请求为%ld", goal->num);
        if (goal->num < 1) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleProgress> goal_handle) {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "接收到任务取消请求");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute(const std::shared_ptr<GoalHandleProgress> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "开始执行任务");
        rclcpp::Rate loop_rate(10.0);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Progress::Feedback>();
        auto result = std::make_shared<Progress::Result>();
        int64_t sum = 0;
        for (int i = 1; (i <= goal->num) && rclcpp::ok(); i++) {
            sum += i;
            if (goal_handle->is_canceling()) {
                result->sum = sum;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "任务取消");
                return;
            }
            feedback->progress = (double_t)i / goal->num;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "连锁反馈中，进度%.2f", feedback->progress);
            loop_rate.sleep();
        }
        if (rclcpp::ok()) {
            result->sum = sum;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "任务完成！");
        }
    }

    void handle_accepted(const std::shared_ptr<GoalHandleProgress> goal_handle) {
        std::thread{std::bind(&MinimalActionServer::execute, this, _1), goal_handle}.detach();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<MinimalActionServer>();
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}