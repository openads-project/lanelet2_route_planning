#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/LaneletPath.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <route_planning_msgs/msg/route_element.hpp>

namespace lanelet2_route_planning {

/**
 * @brief Finds the index of a point in a line string that is closest to another point.
 *
 * If `consider_order`, then only the points behind or ahead of the projection of the given point are considered.
 *
 * @param[in] line_string line string
 * @param[in] point other point
 * @param[in] consider_order whether to consider the order of points in the line string
 * @param[in] behind whether to consider points behind (or ahead of) the given point
 * @return index of closest point
 */
size_t indexOfLineStringPointClosestToPoint(const std::vector<Eigen::Vector2d>& line_string,
                                            const Eigen::Vector2d& point, const bool consider_order = false,
                                            const bool behind = true);

/**
 * @brief Finds the index of a point in a line string that is locally closest to another point.
 *
 * The purpose of this function is not to find the globally closest point, but the locally closest one.
 * By locally, what is meant is that points closer in order to `idx_indication` are preferred over points that may
 * actually be closer in distance.
 *
 * The range of points 10m before and after `idx_indication` is considered.
 * A maximum distance of 10m is used to match to this range of points, otherwise globally closer ones are preferred.
 *
 * If `consider_order`, then only the points behind or ahead of the projection of the given point are considered.
 *
 * @param[in] line_string line string
 * @param[in] point other point
 * @param[in] idx_indication index of point in line string to indicate the range of points to be considered
 * @param[in] consider_order whether to consider the order of points in the line string
 * @param[in] behind whether to consider points behind (or ahead of) the given point
 * @return index of locally closest point
 */
size_t matchPointToLineString(const std::vector<Eigen::Vector2d>& line_string, const Eigen::Vector2d& point,
                              const size_t idx_indication, const bool consider_order = false, const bool behind = true);

/**
 * @brief Takes a closest point in a line string and guarantees that it is behind or ahead of the given point.
 *
 * Behind/ahead is meant physically, not in the order of the line string.
 * The point behind a point at index i is at index i-1.
 *
 * @param[in] line_string line string
 * @param[in] point other point
 * @param[in] idx_closest index of closest point in line string
 * @param[in] behind whether to consider points behind (or ahead of) the given point
 * @return
 */
size_t considerOrderForPointMatchedToLineString(const std::vector<Eigen::Vector2d>& line_string,
                                                const Eigen::Vector2d& point, const size_t idx_closest,
                                                const bool behind);

/**
 * @brief Identifies a lane change based on the distance between two reference line points.
 *
 * Assumes that the two points are part of an equidistant reference line.
 * If the distance between the two points is greater than the sampling distance, a lane change is assumed.
 *
 * @param[in] point point on reference line
 * @param[in] next_point next point on reference line
 * @param[in] sampling_distance expected distance between points
 * @return whether a lane change is identified
 */
bool changesLaneFromPointToPoint(const Eigen::Vector2d& point, const Eigen::Vector2d& next_point,
                                 const double sampling_distance);

/**
 * @brief Finds lanelets adjacent to the left or right of a given lanelet.
 *
 * RelationType::Left and RelationType::AdjacentLeft are used for left adjacent lanelets, right vice versa.
 *
 * @param[in] lanelet lanelet
 * @param[in] route route
 * @param[in] left whether to find left or right adjacent lanelets
 * @return adjacent lanelets
 */
std::vector<lanelet::ConstLanelet> adjacentLeftOrRightLanelets(const lanelet::ConstLanelet& lanelet,
                                                               const lanelet::routing::Route& route, bool left);

/**
 * @brief Projected lanelet points.
 */
struct ProjectedLaneletPoints {
  Eigen::Vector2d left_bound_point;   ///< projected left bound point
  Eigen::Vector2d centerline_point;   ///< projected centerline point
  Eigen::Vector2d right_bound_point;  ///< projected right bound point
};

/**
 * @brief Projects a point to the centerline and bounds of a set of lanelets.
 *
 * For projection, projectPointToLineStringAlongNormal is used.
 *
 * @param[in] point point
 * @param[in] prev_point previous point
 * @param[in] next_point next point
 * @param[in] lanelets lanelets
 * @param[in] logger logger (for error logging)
 * @return projected points
 */
std::vector<ProjectedLaneletPoints> projectPointToLaneletLines(
    const Eigen::Vector2d& point, const Eigen::Vector2d& prev_point, const Eigen::Vector2d& next_point,
    const std::vector<lanelet::ConstLanelet>& lanelets,
    const rclcpp::Logger& logger = rclcpp::get_logger("lanelet2_route_planning"));

/**
 * @brief Computes the offset of lane element indices from current to next route element.
 *
 * The algorithm works as follows:
 * - find adjacent lanelets of current lanelet
 * - find adjacent lanelets of next lanelet (lanelet of next point)
 * - try to match the ID of the lanelet following the current one to the next lanelet and its adjacent lanelets
 * - if no match is found, try to match the ID of the lanelets following adjacent lanelets of the current lanelet to
 *   the next lanelet and its adjacent lanelets
 * - keep following lanelets for up to 3 iterations to account for lanelets that are skipped during sampling
 *
 * If lane element j of the next route element follows on lane element i of the current route element, the returned
 * offset is (j-i).
 *
 * @param[in] lanelet current lanelet
 * @param[in] lanelet_of_next_point lanelet of next point (on next route element)
 * @param[in] route route
 * @param[in] routing_graph routing graph
 * @return following lane index offset
 */
std::optional<int> computeFollowingLaneIdxOffset(const lanelet::ConstLanelet& lanelet,
                                                 const lanelet::ConstLanelet& lanelet_of_next_point,
                                                 const lanelet::routing::Route& route,
                                                 const lanelet::routing::RoutingGraphUPtr& routing_graph);

/**
 * @brief Create a minimal route element message.
 *
 * A minimal valid route element only has a single lane element with a reference pose.
 *
 * @param[in] position position
 * @param[in] orientation orientation
 * @param[in] s accumulated distance along route
 * @param[in] will_change_suggested_lane whether the lane will change to the next route element
 * @param[in] speed_limit speed limit
 * @return minimal route element
 */
route_planning_msgs::msg::RouteElement createMinimalRouteElement(const geometry_msgs::msg::Point& position,
                                                                 const geometry_msgs::msg::Quaternion& orientation,
                                                                 double s = 0.0,
                                                                 bool will_change_suggested_lane = false,
                                                                 uint8_t speed_limit = 0);

/**
 * @brief Helper type for a sequence of three points.
 */
struct PointSequence {
  Eigen::Vector2d prev;     ///< previous point
  Eigen::Vector2d current;  ///< current point
  Eigen::Vector2d next;     ///< next point
  PointSequence(const Eigen::Vector2d& prev, const Eigen::Vector2d& current, const Eigen::Vector2d& next)
      : prev(prev), current(current), next(next) {}
};

/**
 * @brief Extracts drivable space boundaries for a route element.
 *
 * Drivable space is defined as the area around the current point that is physically drivable.
 * To extract it, the intersection of the normal to the current point with all existing map line strings is computed.
 * The outermost intersection point that is not drivable anymore is considered the boundary.
 * If no such point is found, the maximum distance is used.
 *
 * @param[in] line_string_layer map line string layer
 * @param[in] point_sequence point sequence (should be centerline of main lanelet)
 * @param[in] max_distance maximum distance to left/right drivable space bounds, if not otherwise restricted
 * @return left and right drivable space bounds
 */
std::pair<Eigen::Vector2d, Eigen::Vector2d> extractDrivableSpace(const lanelet::LineStringLayer& line_string_layer,
                                                                 const PointSequence& point_sequence,
                                                                 const double max_distance);

/**
 * @brief Checks if lanelet line string has a type that is considered drivable.
 *
 * Drivable in the sense of being able to physically cross it, not whether it is allowed to cross it.
 *
 * The following types are considered drivable:
 * - line_thin
 * - line_thick
 * - virtual
 * - zebra_marking
 * - bike_marking
 * - pedestrian_marking
 * - stop_line
 * - traffic_light
 * - curbstone (only if subtype is 'low')
 * - roadpainting
 * - lane_center
 * - centerline
 *
 * Additionally, the extra attribute 'HoldingLine' is considered drivable.
 *
 * @param[in] line_string line string
 * @return whether line string is drivable
 */
bool isLineStringDrivable(const lanelet::ConstLineString3d& line_string);

/**
 * @brief Return type of extractRegulatoryElements.
 */
struct ExtractRegulatoryElementsResult {
  std::vector<route_planning_msgs::msg::RegulatoryElement>
      regulatory_element_msgs;                   ///< regulatory element messages for route element
  std::vector<uint8_t> regulatory_element_idcs;  ///< indices of regulatory elements belonging to main lane
  std::vector<std::vector<uint8_t>>
      adjacent_left_regulatory_element_idcs;  ///< indices of regulatory elements belonging to left adjacent lanes
  std::vector<std::vector<uint8_t>>
      adjacent_right_regulatory_element_idcs;  ///< indices of regulatory elements belonging to right adjacent lanes
};

/**
 * @brief Extracts regulatory element information for a route element.
 *
 * Regulatory elements are queried from a lanelet and its adjacent lanelets. They are only considered if their reference
 * line intersects with the given point sequence, which should be the centerline of the main lanelet. This way,
 * regulatory elements are assignable to the closest route element. Note that the assignment to adjacent lanes is also
 * based on the intersection with the single given point sequence.
 *
 * @param[in] lanelet lanelet
 * @param[in] adjacent_left_lanelets left adjacent lanelets
 * @param[in] adjacent_right_lanelets right adjacent lanelets
 * @param[in] point_sequence point sequence (should be centerline of main lanelet)
 * @return regulatory element information
 */
ExtractRegulatoryElementsResult extractRegulatoryElements(
    const lanelet::ConstLanelet& lanelet, const std::vector<lanelet::ConstLanelet>& adjacent_left_lanelets,
    const std::vector<lanelet::ConstLanelet>& adjacent_right_lanelets, const PointSequence& point_sequence);

/**
 * @brief Extracts the reference/effect line of a regulatory element.
 *
 * Only the first reference line of the regulatory element is considered.
 * Only the end points of that reference line are considered.
 *
 * @param[in] regulatory_element regulatory element
 * @return reference line
 */
std::optional<std::array<geometry_msgs::msg::Point, 2>> regulatoryElementReferenceLine(
    const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element);

/**
 * @brief Extracts the sign/signal positions of a regulatory element.
 *
 * Only the first point of referenced line strings is considered.
 *
 * @param[in] regulatory_element regulatory element
 * @return positions
 */
std::vector<geometry_msgs::msg::Point> regulatoryElementPositions(
    const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element);

/**
 * @brief Extracts the type and meta value of a regulatory element.
 *
 * https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/RegulatoryElementTagging.md
 *
 * Type and meta value as in route_planning_msgs::msg::RegulatoryElement.
 *
 * @param[in] regulatory_element regulatory element
 * @return type and meta value
 */
std::pair<uint8_t, uint8_t> regulatoryElementType(
    const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element);

/**
 * @brief Extracts the speed limit of a regulatory element of subtype 'speed_limit'.
 *
 * As defined in route_planning_msgs::msg::RegulatoryElement.
 *
 * @param[in] regulatory_element regulatory element
 * @return speed limit [km/h]
 */
uint8_t regulatoryElementSpeedLimit(const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element);

/**
 * @brief Extracts the lane boundary type of a lanelet line.
 *
 * Lane boundary type as in route_planning_msgs::msg::LaneBoundary.
 *
 * @param[in] line lanelet line
 * @return lane boundary type
 */
uint8_t laneBoundaryType(const lanelet::ConstLineString2d& line);

/**
 * @brief Extracts the speed limit of a lanelet.
 *
 * As defined in route_planning_msgs::msg::RegulatoryElement.
 *
 * @param[in] lanelet lanelet
 * @return speed limit [km/h]
 */
uint8_t speedLimit(const lanelet::ConstLanelet& lanelet);

/**
 * @brief Get traffic rules.
 *
 * @return traffic rules
 */
lanelet::traffic_rules::TrafficRulesPtr getTrafficRules();

/**
 * @brief Find lanelet at arbitrary point.
 *
 * Finds closest passable lanelet within 10m of the given point.
 * Only considers the 5 closest lanelets.
 *
 * @param[in] point point
 * @param[in] map map
 * @param[in] traffic_rules traffic rules
 * @return matching lanelet
 */
std::optional<lanelet::ConstLanelet> laneletAtPoint(
    const Eigen::Vector2d& point, const lanelet::LaneletMapConstPtr& map,
    const std::optional<lanelet::traffic_rules::TrafficRulesPtr> traffic_rules = std::nullopt);

/**
 * @brief Follows a lanelet's and following lanelets' centerline for a given distance.
 *
 * If lanelets cannot be followed anymore, returns the last lanelet.
 *
 * @param[in] routing_graph routing graph
 * @param[in] lanelet current lanelet
 * @param[in] position position on current lanelet
 * @param[in] distance distance to follow lanelets (may be negative to follow in opposite direction)
 * @return followed lanelet
 */
lanelet::ConstLanelet followLaneletsAlongRoutingGraph(const lanelet::routing::RoutingGraphUPtr& routing_graph,
                                                      const lanelet::ConstLanelet& lanelet,
                                                      const Eigen::Vector2d& position, const double distance);

/**
 * @brief Return type of resampleCenterlinesAlongPath.
 */
struct ResampleCenterlinesAlongPathResult {
  std::vector<Eigen::Vector2d> centerline;   ///< resampled centerline
  std::vector<size_t> lanelet_idx_by_point;  ///< lanelet index in path for each point
};

/**
 * @brief Equidistantly resamples lanelet centerlines along a path to one joint centerline.
 *
 * @param[in] path path
 * @param[in] delta_s sampling distance
 * @param[in] monotonically whether to ensure that the centerline cannot make turns greater than 90 degrees
 * @return resampled centerline and lanelet index by point
 */
ResampleCenterlinesAlongPathResult resampleCenterlinesAlongPath(const lanelet::routing::LaneletPath& path,
                                                                const double delta_s, bool monotonically);

/**
 * @brief Estimate remaining time for a route based on speed limits.
 *
 * @param[in] route_elements route elements
 * @param[in] reference_speed reference speed if speed limit is not set [m/s]
 * @return estimated remaining time [s]
 */
double estimateRemainingTime(const std::vector<route_planning_msgs::msg::RouteElement>& route_elements,
                             const double reference_speed = 50.0 / 3.6);

/**
 * @brief Postprocesses a route message, filling missing information that can be inferred from other message contents.
 *
 * This includes:
 * - orientation of each lane element's reference pose based on preceding and following points
 *
 * @param[in,out] route_msg route message to postprocess
 */
void postprocessRouteMessage(route_planning_msgs::msg::Route& route_msg);

}  // namespace lanelet2_route_planning