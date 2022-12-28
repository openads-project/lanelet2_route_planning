#include "rclcpp/rclcpp.hpp"

#include "lanelet2_map_interface/lanelet2_map_interface.hpp"

using namespace std::chrono_literals;

class GlobalPlanner : public rclcpp::Node
{
    public:
        GlobalPlanner();
        void initializeMapInterface();
        bool map_if_initialized_ = false;
        
    private:
        LL2MapInterface *ll2if_;
};