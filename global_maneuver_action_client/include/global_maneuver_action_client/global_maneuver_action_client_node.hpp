#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "route_planning_msgs/action/global_maneuver.hpp"
#include <lanelet2_map_interface/lanelet2_map_interface.hpp>

namespace global_maneuver_action_client {

// templates for parameter loading
template <typename C> struct is_vector : std::false_type {};    
template <typename T,typename A> struct is_vector< std::vector<T,A> > : std::true_type {};    
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;

class GlobalManeuverActionClient : public rclcpp::Node {
  using GlobalManeuver = route_planning_msgs::action::GlobalManeuver;
  using GoalHandleGlobalManeuver = rclcpp_action::ClientGoalHandle<GlobalManeuver>;

 public:
  GlobalManeuverActionClient();

 private:
  // input topics
  const std::string kGoalPoseTopic = "~/goal_pose";

  void setup();

  void goalPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void generateRandomGoal();
  void sendGoal(geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // action callbacks
  void goalResponseCallback(const GoalHandleGlobalManeuver::SharedPtr& goal_handle);
  void feedbackCallback(GoalHandleGlobalManeuver::SharedPtr goal_handle,
                        const std::shared_ptr<const GlobalManeuver::Feedback> feedback);
  void resultCallback(const GoalHandleGlobalManeuver::WrappedResult& result);


  // parameter loading
  template <typename T>
  void declareAndLoadParameter(const std::string &name, T &member_param, const std::string &description,
                               const bool add_to_auto_reconfigurable_params = true, const bool is_required = false,
                               const bool read_only = false, const std::optional<T> &from_value = std::nullopt,
                               const std::optional<T> &to_value = std::nullopt,
                               const std::optional<T> &step_value = std::nullopt,
                               const std::string &additional_constraints = "");
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>> auto_reconfigurable_params_;
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  // parameter defaults
  bool random_planning_ = false;
  std::string map_server_name_ = "ll2_map_server";

  // subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // action client
  rclcpp_action::Client<GlobalManeuver>::SharedPtr action_client_;
  std::shared_future<GoalHandleGlobalManeuver::SharedPtr> goal_handle_future_;

  //lanelet2 map interface
  std::unique_ptr<LL2MapInterface> ll2if_;
};

}  // namespace global_maneuver_action_client