#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicSubscription : public rclcpp::Node{
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    public:
        TopicSubscription() : Node("TOpicSubscription"){
            subscription_ = this->create_subscription<std_msgs::msg::String>("topic_std" , 10 ,
                std::bind(&TopicSubscription::CinNum , this ,std::placeholders::_1));
        }
    
    private:
        void CinNum (const std_msgs::msg::String& msg) const{
            RCLCPP_INFO(this->get_logger() , "subscription :%s" , msg.data.c_str());
        }
};

int main(int argc , char** argv){
    rclcpp::init(argc , argv);
    auto node = std::make_shared<TopicSubscription>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}