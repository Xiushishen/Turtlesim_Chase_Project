#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class SmartPhoneNode : public rclcpp::Node
{
public:
    SmartPhoneNode() : Node("smartphone")
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::String>("robot_news", 10,
                      std::bind(&SmartPhoneNode::callbackRobotNews, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Smartphone has been started.");
    }
private:
    void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }

    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartPhoneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}