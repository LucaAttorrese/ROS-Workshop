#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//Define a class Test_sub which inherits from rclcpp::Node (standard node class)
class Test_sub : public rclcpp::Node{
public:
    Test_sub() : Node("Test_sub")
    {
        //Create the subscriber
        subscription_ = this->create_subscription<std_msgs::msg::String>("chat", 10, std::bind(&Test_sub::message_callback, this, std::placeholders::_1));
    }

private:
    //Function called every time it receives a msg
    void message_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        //Print the msg on the terminal
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    //Initialize ros2
    rclcpp::init(argc, argv);

    //Initialize the node Test_sub
    auto node = std::make_shared<Test_sub>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
