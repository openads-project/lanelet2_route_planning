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
#include <route_planning_msgs/msg/route_element.hpp>

namespace lanelet2_route_planning {

/**
 * @brief Find index of point in line string that is closest to another point.
 *
 * @param[in] line_string line string
 * @param[in] point other point
 * @return index of closest point
 */
size_t indexOfLineStringPointClosestToPoint(const std::vector<Eigen::Vector2d>& line_string,
                                            const Eigen::Vector2d& point);

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
 * @return projected points
 */
std::vector<ProjectedLaneletPoints> projectPointToLaneletLines(const Eigen::Vector2d& point,
                                                               const Eigen::Vector2d& prev_point,
                                                               const Eigen::Vector2d& next_point,
                                                               const std::vector<lanelet::ConstLanelet>& lanelets);

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
int computeFollowingLaneIdxOffset(const lanelet::ConstLanelet& lanelet,
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
 * @brief Extracts regulatory element information for a route element.
 *
 * Regulatory elements are queried from the lanelet.
 * They are only considered if their reference line intersects with the line from point to next point or previous point to point.
 * This way, regulatory elements are assignable to the closest route element.
 *
 * @param[in] lanelet lanelet
 * @param[in] point point
 * @param[in] prev_point previous point
 * @param[in] next_point next point
 * @return regulatory elements
 */
std::vector<route_planning_msgs::msg::RegulatoryElement> extractRegulatoryElements(const lanelet::ConstLanelet& lanelet,
                                                                                   const Eigen::Vector2d& point,
                                                                                   const Eigen::Vector2d& prev_point,
                                                                                   const Eigen::Vector2d& next_point);

/**
 * @brief Extracts the reference/effect line of a regulatory element.
 *
 * Only the first reference line of the regulatory element is considered.
 * Only the end points of that reference line are considered.
 *
 * @param[in] regulatory_element
 * @return reference line
 */
std::optional<std::array<geometry_msgs::msg::Point, 2>> regulatoryElementReferenceLine(
    const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element);

/**
 * @brief Extracts the sign/signal positions of a regulatory element.
 *
 * Only the first point of referenced line strings is considered.
 *
 * @param[in] regulatory_element
 * @return positions
 */
std::vector<geometry_msgs::msg::Point> regulatoryElementPositions(
    const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element);

/**
 * @brief Extracts the type and meta value of a regulatory element.
 *
 * Type and meta value as in route_planning_msgs::msg::RegulatoryElement.
 *
 * @param[in] regulatory_element
 * @return type and meta value
 */
std::pair<uint8_t, uint8_t> regulatoryElementType(
    const std::shared_ptr<const lanelet::RegulatoryElement>& regulatory_element);

/**
 * @brief Extracts the lane boundary type of a lanelet line.
 *
 * Lane boundary type as in route_planning_msgs::msg::LaneBoundary.
 *
 * @param[in] line
 * @return lane boundary type
 */
uint8_t laneBoundaryType(const lanelet::ConstLineString2d& line);

/**
 * @brief Extracts the speed limit of a lanelet.
 *
 * Returns 0 if no speed limit is set or speed limit is not mandatory.
 *
 * @param[in] lanelet
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
 * @param[in] route_msg postprocessed route message
 */
void postprocessRouteMessage(route_planning_msgs::msg::Route& route_msg);

}  // namespace lanelet2_route_planning