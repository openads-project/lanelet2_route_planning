#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class GlobalPlanner : public rclcpp::Node
{
    public:
        GlobalPlanner();
        
    private:
        void timer_callback();
        rclcpp::TimerBase::SharedPtr timer_;
};