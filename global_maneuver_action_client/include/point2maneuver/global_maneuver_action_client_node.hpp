#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "route_planning_msgs/action/global_maneuver.hpp"

class GlobalManeuverActionClient : public rclcpp::Node
{
    public:
        GlobalManeuverActionClient();

    private:

        // Subscriptions
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;

        // Action Client
        rclcpp_action::Client<route_planning_msgs::action::GlobalManeuver>::SharedPtr action_client_;
        std::shared_future<rclcpp_action::ClientGoalHandle<route_planning_msgs::action::GlobalManeuver>::SharedPtr> goal_handle_future_;

        // Callbacks
        void pointCallback(geometry_msgs::msg::PointStamped::SharedPtr msg);
};