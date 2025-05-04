#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//Define a class Test_pub which inherits from rclcpp::Node (standard node class)
class Test_pub : public rclcpp::Node{
public:
    Test_pub() : Node("Test_pub")
    {
		//Create the publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("chat", 10);

		//Define a timer that publish the message every 5 seconds
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&Test_pub::publish_message, this));
    }

private:
    void publish_message()
    {
		//Set the content of the message as Hello world
        auto message = std_msgs::msg::String();
        message.data = "Hello world!";

		//Prints on the terminal the pub action
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

		//The message is published
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
	//Initialize ros2
    rclcpp::init(argc, argv);

	//Initialize the node Test_pub
    auto node = std::make_shared<Test_pub>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}