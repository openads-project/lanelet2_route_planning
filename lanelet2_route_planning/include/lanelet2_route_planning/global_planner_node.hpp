#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

#include "nav_msgs/msg/odometry.hpp"

#include "lanelet2_map_interface/lanelet2_map_interface.hpp"
#include "lanelet2_route_planning_ifs/action/global_maneuver.hpp"
#include "lanelet2_utilities/lanelet2_utils.hpp"

using namespace std::chrono_literals;

class GlobalPlanner : public rclcpp::Node
{
    public:
        GlobalPlanner();
        void initializeMapInterface();
        
        
    private:
        // Variables
        LL2MapInterface *ll2if_;
        nav_msgs::msg::Odometry ego_pose_;
        
        // Timer
        rclcpp::TimerBase::SharedPtr startup_timer_;

        // Subscriptions
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr map_pose_sub_;

        // Actions
        rclcpp_action::Server<lanelet2_route_planning_ifs::action::GlobalManeuver>::SharedPtr maneuver_action_server_;
        lanelet2_route_planning_ifs::action::GlobalManeuver::Feedback::SharedPtr maneuver_feedback_;
        lanelet2_route_planning_ifs::action::GlobalManeuver::Result::SharedPtr maneuver_result_;

        // Function Definitions
        // global_planner_node.cpp
        void initializeGlobalPlanner();
        bool egoPositionSanityCheck();
        
        // maneuver_action_fcns.cpp
        rclcpp_action::GoalResponse actionHandleGoal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const lanelet2_route_planning_ifs::action::GlobalManeuver::Goal> goal);

        rclcpp_action::CancelResponse actionHandleCancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_ifs::action::GlobalManeuver>> goal_handle);

        void actionHandleAccepted(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_ifs::action::GlobalManeuver>> goal_handle);

        void actionExecute(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_ifs::action::GlobalManeuver>> goal_handle);

        // callbacks.cpp
        void mapPoseCallback(nav_msgs::msg::Odometry::SharedPtr msg);

};