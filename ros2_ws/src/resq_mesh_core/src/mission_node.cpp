#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MissionNode : public rclcpp::Node
{
public:
    MissionNode() : Node("mission_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "mission_status", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MissionNode::publish_status, this));
    }

private:
    void publish_status()
    {
        auto message = std_msgs::msg::String();

        message.data = "RESQ-MESH mission system online";

        RCLCPP_INFO(this->get_logger(), "Publishing: %s",
                    message.data.c_str());

        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MissionNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
