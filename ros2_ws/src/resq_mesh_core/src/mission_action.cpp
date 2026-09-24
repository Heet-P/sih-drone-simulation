#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class MissionAction : public rclcpp::Node
{
public:
    using Mission = example_interfaces::action::Fibonacci;
    using GoalHandleMission = rclcpp_action::ServerGoalHandle<Mission>;

    MissionAction() : Node("mission_action")
    {
        action_server_ = rclcpp_action::create_server<Mission>(
            this,
            "mission_action",
            std::bind(
                &MissionAction::handle_goal,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            std::bind(
                &MissionAction::handle_cancel,
                this,
                std::placeholders::_1),
            std::bind(
                &MissionAction::handle_accepted,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Mission action server is ready");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Mission::Goal> goal)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Mission received with target: %d",
            goal->order);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMission>)
    {
        RCLCPP_INFO(this->get_logger(), "Mission cancellation requested");

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<GoalHandleMission> goal_handle)
    {
        std::thread(
            std::bind(
                &MissionAction::execute,
                this,
                std::placeholders::_1),
            goal_handle).detach();
    }

    void execute(
        const std::shared_ptr<GoalHandleMission> goal_handle)
    {
        const auto goal = goal_handle->get_goal();

        auto feedback = std::make_shared<Mission::Feedback>();
        auto result = std::make_shared<Mission::Result>();

        feedback->sequence.push_back(0);

        for (int i = 1; i <= goal->order; ++i)
        {
            if (goal_handle->is_canceling())
            {
                result->sequence = feedback->sequence;
                goal_handle->canceled(result);

                RCLCPP_INFO(
                    this->get_logger(),
                    "Mission cancelled");

                return;
            }

            feedback->sequence.push_back(
                feedback->sequence[i - 1] +
                (i > 1 ? feedback->sequence[i - 2] : 1));

            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(
                this->get_logger(),
                "Mission progress: %d/%d",
                i,
                goal->order);

            std::this_thread::sleep_for(1s);
        }

        result->sequence = feedback->sequence;

        goal_handle->succeed(result);

        RCLCPP_INFO(
            this->get_logger(),
            "Mission completed");
    }

    rclcpp_action::Server<Mission>::SharedPtr action_server_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MissionAction>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
