//*******************************
//Action 三要素（Goal/Feedback/Result）
//*******************************
// ******************************
// * 1.
// *
// *
// *
// *
// *
// *
// ******************************
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"  //action 核心库
#include "base_interface_demo/action/progress.hpp"
using namespace std::placeholders;  //简化bind占位符 _1,_2
using base_interface_demo::action::Progress;
using GoalHandleProgress = rclcpp_action::ServerGoalHandle<Progress>; //别名 rclcpp_action::ServerGoalHandle<Progress>：操作Goal的唯一句柄

class MinimalActionServer : public rclcpp::Node{
    private:
        rclcpp_action::Server<Progress>::SharedPtr action_server_;
    public:
        explicit MinimalActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("minimal_action_server",options){
        //explicit 禁止隐式转换 ；const rclcpp::NodeOptions& options = rclcpp::NodeOptions() 中rclcpp::NodeOptions是节点状态类 rclcpp::NodeOptions()使用默认的节点状态
        //Node(...,options) 将节点状态交由父类构建
            //创建动作服务器
            this->action_server_ = rclcpp_action::create_server<Progress>(this , "get_sum", //设置动作服务器的处理方法，调用handle_goal函数，再根据不同结果掉用不同函数
            std::bind(&MinimalActionServer::handle_goal , this , _1 , _2),
            std::bind(&MinimalActionServer::handle_cancel , this , _1),
            std::bind(&MinimalActionServer::handle_accepted , this , _1));
            RCLCPP_INFO(this->get_logger() , "动作服务端创建，等待请求！");
            }
    private:
        //处理请求数据
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid , std::shared_ptr<const Progress::Goal> goal){
        //rclcpp_action::GoalResponse枚举类型：REJECT（拒绝请求）、ACCEPT_AND_EXECUTE（接受并立即执行）、ACCEPT_AND_DEFER（接受但需手动触发后执行）
        //GoalUUID：goal请求的唯一标识
            (void)uuid; //抑制编译器的 “未使用参数” 警告
            RCLCPP_INFO(this->get_logger(),"接收到动作客户端请求，请求为%ld" , goal->num);
            if(goal->num < 1){
                return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; //继续调用handle_accept 函数
        }





        //处理取消任务请求
        //当客户端请求取消后调用
        //取消后的返回值终止execute函数
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleProgress> goal_handle){
        //rclcpp_action::CancelResponse：ACCEPT（同意取消，仅标记，具体取消操作由execute实现）、REJECT（拒绝）
            (void)goal_handle; //抑制编译器的 “未使用参数” 警告
            RCLCPP_INFO(this->get_logger(),"接收到任务取消请求");
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        void execute(const std::shared_ptr<GoalHandleProgress> goal_handle){
            RCLCPP_INFO(this->get_logger(), "开始执行任务");
            rclcpp::Rate loop_rate(10.0); //设置循环频率，即两个loop_rate.sleep()之间的时间间隔
            const auto goal = goal_handle->get_goal(); //从goal_handle 中取出goal
            auto feedback = std::make_shared<Progress::Feedback>();
            auto result = std::make_shared<Progress::Result>();
            int64_t sum = 0;
            for(int i = 1;(i <= goal->num) && rclcpp::ok(); i++){ 
                sum += i;
                if(goal_handle->is_canceling()){ //获得取消指令
                    result->sum = sum;  //保存结果
                    goal_handle->canceled(result); //通知取消并附带取消时的结果
                    RCLCPP_INFO(this->get_logger(),"任务取消");
                    return ;
                }
                feedback->progress = (double_t)i / goal->num; //(double_t)i：把整数 i 转换成浮点型
                goal_handle->publish_feedback(feedback); //实时汇报进度
                RCLCPP_INFO(this->get_logger(),"连锁反馈中，进度%.2f",feedback->progress);
                loop_rate.sleep();
            }
            if(rclcpp::ok()){
                result->sum = sum;
                goal_handle->succeed(result);//返回成功执行完的结果
                RCLCPP_INFO(this->get_logger(),"任务完成！");
            }
        }

        //生成连续反馈
        void handle_accepted(const std::shared_ptr<GoalHandleProgress> goal_handle){
            //当handle_goal接受Goal请求后调用
            std::thread{std::bind(&MinimalActionServer::execute , this , _1),goal_handle}.detach();
            //新建线程std::thread 
            //goal_handle 作为参数
            //.detach() 让新线程独立运行
        }
};

int main(int argc ,char** argv){
    rclcpp::init(argc , argv);
    auto action_server = std::make_shared<MinimalActionServer>();
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}




