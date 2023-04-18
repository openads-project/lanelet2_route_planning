#ifndef LL2_ROUTE_PLANNING_DATATYPES_HPP
#define LL2_ROUTE_PLANNING_DATATYPES_HPP

// Includes
#include <lanelet2_core/primitives/LineString.h>

// Datatype Definition
namespace Lanelet2RoutePlanningDatatypes
{
    struct LaneletLaneSection {
        float accumulated_s; // Accumulated length of lane at end of this section
        u_int16_t route_index; // Index in LaneletLaneHierarchy.lane_hierarchy (in which section of the route)
        u_int16_t spatial_index; // Index in LaneletLaneHierarchy.lane_hierarchy[route_index].neighboring_lanelets (which spatial position (right -> left) of the section)
    };

    struct LaneletExtended {
        int64_t lanelet_id;
        int16_t lane_id;
        float v_max;
    };

    struct LaneletLane {
        std::vector<LaneletLaneSection> lane_sections;
        lanelet::BasicLineString2d line;
    };

    struct LaneletLaneHierarchy {
        std::vector<LaneletExtended> neighboring_lanelets; // From most right (index 0) to most left
        int16_t shortest_path_index; // Index in neighboring_lanelets of the shortest path entry
    };

    struct LaneletLaneNetwork {
        std::vector<LaneletLaneHierarchy> lane_hierarchy; // Contains a spatial ordering of all lanelets of the route (from right to left per step) and maps them their lane id
        std::vector<LaneletLane> lanes; // Contains mapping from lane id to all belonging lanelet ids + a smoothed line string for each line
    };    

} // namespace Lanelet2RoutePlanningDatatypes

#endif // LL2_ROUTE_PLANNING_DATATYPES_HPP

