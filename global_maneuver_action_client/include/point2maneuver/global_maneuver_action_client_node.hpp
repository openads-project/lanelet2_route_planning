#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "route_planning_msgs/action/global_maneuver.hpp"

namespace global_maneuver_action_client {

class GlobalManeuverActionClient : public rclcpp::Node
{
    using GlobalManeuver = route_planning_msgs::action::GlobalManeuver;
    using GoalHandleGlobalManeuver = rclcpp_action::ClientGoalHandle<GlobalManeuver>;

    public:
        GlobalManeuverActionClient();

    private:
        void sendGoal(geometry_msgs::msg::PointStamped::SharedPtr msg);

        // action callbacks
        void goal_response_callback(std::shared_future<GoalHandleGlobalManeuver>::SharedPtr> future);
        void feedback_callback(GoalHandleGlobalManeuver::SharedPtr goal_handle, const std::shared_ptr<const GlobalManeuver::Feedback> feedback);
        void result_callback(const GoalHandleGlobalManeuver::WrappedResult& result);

    private:
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;

        // action client
        rclcpp_action::Client<GlobalManeuver>::SharedPtr action_client_;
        std::shared_future<GoalHandleGlobalManeuver::SharedPtr> goal_handle_future_;
};

}