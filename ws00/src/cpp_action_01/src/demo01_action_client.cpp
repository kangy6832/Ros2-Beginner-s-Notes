/*
${workspaceFolder}/**
/opt/ros/humble/include/**
/opt/ros/humble/include/std_msgs/**
/opt/ros/humble/include/rclcpp/**
/opt/ros/humble/include/rcl/**
/opt/ros/humble/include/rclcpp_action/**
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interface_demo/action/progress.hpp"
using base_interface_demo::action::Progress;
using GoalHandleProgress = rclcpp_action::ClientGoalHandle<Progress>; //句柄
using namespace std::placeholders;

class MinimalActionClient : public rclcpp::Node{
    private:
        rclcpp_action::Client<Progress>::SharedPtr client_ptr_;
    public:
        explicit MinimalActionClient(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()):Node("minimal_action_client" , node_options){
            //创建动作客户端
            this->client_ptr_ = rclcpp_action::create_client<Progress>(this , "get_sum");
        }

        //发送请求
        void send_goal(int64_t num){
            if(!this->client_ptr_){ //this->client_ptr_:指针为空则返回false
                RCLCPP_ERROR(this->get_logger(),"动作客户端未被初始化");
            }
            if(!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))){
                RCLCPP_ERROR(this->get_logger(),"服务器连接失败");
                return ;
            }
            auto goal_msg = Progress::Goal();
            goal_msg.num = num;
            RCLCPP_INFO(this->get_logger(),"发送请求数据");

            auto send_goal_options = rclcpp_action::Client<Progress>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&MinimalActionClient::goal_response_callback , this ,_1);
            send_goal_options.feedback_callback = std::bind(&MinimalActionClient::feedback_callback , this ,_1 , _2);
            send_goal_options.result_callback = std::bind(&MinimalActionClient::result_callback , this , _1);
            auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg , send_goal_options);
        }
    private:
        void goal_response_callback(GoalHandleProgress::SharedPtr goal_handle){
            if(!goal_handle){ //goal_handle是否为空
                RCLCPP_ERROR(this->get_logger(),"目标请求被服务器拒绝");
            } else {
                RCLCPP_INFO(this->get_logger(),"目标被接受，等待结果中");
            }
        }
        void feedback_callback(GoalHandleProgress::SharedPtr , const std::shared_ptr<const Progress::Feedback> feedback){
            int32_t progress = (int32_t)(feedback->progress * 100);
            RCLCPP_INFO(this->get_logger(),"当前进度：%d%%" , progress);
        }

        //处理最终响应
        void result_callback(const GoalHandleProgress::WrappedResult& result){
            switch(result.code){
                case rclcpp_action::ResultCode::SUCCEEDED:
                break;
                case rclcpp_action::ResultCode::CANCELED:
                    {
                        RCLCPP_ERROR(this->get_logger() , "任务被取消");
                        return ;
                    }
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(),"任务被中止");
                    return ;
                default:
                    RCLCPP_ERROR(this->get_logger(),"未知异常");
                    return ;
            };
            RCLCPP_INFO(this->get_logger(),"任务执行完毕，最终结果：%ld",result.result->sum);
        }
};

int main(int argc ,char** argv){
    rclcpp::init(argc , argv);
    auto action_client = std::make_shared<MinimalActionClient>();
    action_client->send_goal(10);
    rclcpp::spin(action_client);
    rclcpp::shutdown();
    return 0;
}


