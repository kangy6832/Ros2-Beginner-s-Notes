#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <interface_demo/action/progress.hpp>
using namespace std::placeholders;
using interface_demo::action::Progress;
using GoalHandleProgress = rclcpp_action::ServerGoalHandle<Progress>;

class MinimalActionServer : public rclcpp::Node
{
    public:
        MinimalActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("minimal_action_server", options)
        {
        this->action_server_ = rclcpp_action::create_server<Progress>(
            this, "get_sum", std::bind(&MinimalActionServer::handle_goal, this, _1, _2),
            std::bind(&MinimalActionServer::handle_cancel, this, _1),
            std::bind(&MinimalActionServer::handle_accepted, this, _1));
        RCLCPP_INFO(this->get_logger(), "动作服务端创建，等待请求！");
        }

    private:
        rclcpp_action::Server<Progress>::SharedPtr action_server_;
        
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, Progress::Goal::SharedPtr goal){
            if(goal->num < 1){
                return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

    void execute (std::shared_ptr<GoalHandleProgress> goal_handle){
        RCLCPP_INFO(this->get_logger(), "开始执行任务");
        rclcpp::Rate loop_rate(10.0);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Progress::Feedback>();
        auto result = std::make_shared<Progress::Result>();
        int64_t sum = 0;
        for(int i = 1; (i < goal->num) && rclcpp::ok(); i++){
            sum += 1;
            if(goal_handle->is_canceling()){
            result->sum = sum ; // 保存结果
            goal_handle->canceled(result); // 通知取消并附带取消时的结果
            RCLCPP_INFO(this->get_logger(), "cancel");
            return;
            }
            feedback->progress = double_t(sum) / goal->num ;
            goal_handle->publish_feedback(feedback); // 实时汇报进度
            RCLCPP_INFO(this->get_logger(), " ");
            loop_rate.sleep();
        }
        if(rclcpp::ok()){
            result->  =   
            goal_handle->succeed(result); // 返回成功执行完的结果
            RCLCPP_INFO(this->get_logger(), " ");
        }
    }

    };