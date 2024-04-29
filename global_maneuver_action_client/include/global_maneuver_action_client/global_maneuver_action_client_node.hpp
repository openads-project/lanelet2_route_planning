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
        void sendGoal(geometry_msgs::msg::PoseStamped::SharedPtr msg);

        // action callbacks
        void goalResponseCallback(const GoalHandleGlobalManeuver::SharedPtr& goal_handle);
        void feedbackCallback(GoalHandleGlobalManeuver::SharedPtr goal_handle, const std::shared_ptr<const GlobalManeuver::Feedback> feedback);
        void resultCallback(const GoalHandleGlobalManeuver::WrappedResult& result);

    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;

        // action client
        rclcpp_action::Client<GlobalManeuver>::SharedPtr action_client_;
        std::shared_future<GoalHandleGlobalManeuver::SharedPtr> goal_handle_future_;
};

}