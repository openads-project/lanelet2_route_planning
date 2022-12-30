#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

#include "lanelet2_map_interface/lanelet2_map_interface.hpp"
#include "lanelet2_route_planning_ifs/action/global_maneuver.hpp"

using namespace std::chrono_literals;

class GlobalPlanner : public rclcpp::Node
{
    public:
        GlobalPlanner();
        void initializeMapInterface();
        
        
    private:
        void initializeGlobalPlanner();
        LL2MapInterface *ll2if_;

        rclcpp::TimerBase::SharedPtr startup_timer_;
        rclcpp_action::Server<lanelet2_route_planning_ifs::action::GlobalManeuver>::SharedPtr maneuver_action_server_;

        rclcpp_action::GoalResponse actionHandleGoal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const lanelet2_route_planning_ifs::action::GlobalManeuver::Goal> goal);

        rclcpp_action::CancelResponse actionHandleCancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_ifs::action::GlobalManeuver>> goal_handle);

        void actionHandleAccepted(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_ifs::action::GlobalManeuver>> goal_handle);

        void actionExecute(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<lanelet2_route_planning_ifs::action::GlobalManeuver>> goal_handle);

        lanelet2_route_planning_ifs::action::GlobalManeuver::Feedback::SharedPtr maneuver_feedback_;
        lanelet2_route_planning_ifs::action::GlobalManeuver::Result::SharedPtr maneuver_result_;



};