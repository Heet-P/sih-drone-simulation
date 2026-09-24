#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

class MissionService : public rclcpp::Node
{
public:
    MissionService() : Node("mission_service")
    {
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "start_mission",
            std::bind(
                &MissionService::handle_request,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Mission service is ready");
    }

private:
    void handle_request(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            response->success = true;
            response->message = "Mission accepted";

            RCLCPP_INFO(
                this->get_logger(),
                "Mission START requested");
        }
        else
        {
            response->success = true;
            response->message = "Mission stopped";

            RCLCPP_INFO(
                this->get_logger(),
                "Mission STOP requested");
        }
    }

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MissionService>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
