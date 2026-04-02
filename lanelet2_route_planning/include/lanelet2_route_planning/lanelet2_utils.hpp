// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

// TODO: Refactor these old functions from the deprecated lanelet2_utilities
//       package and properly integrate them into the rest of the codebase.

#pragma once

#include <cmath>
#include <vector>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <geometry_msgs/msg/point.hpp>

using namespace lanelet;
using namespace lanelet::traffic_rules;

class Lanelet2Utilities {
 public:
  /**
   * @brief returns the rough heading of the given lanelet at the given length
   * @param laneletLine - ConstLineString3d
   * @param length, length at which to look at - double
   * @return double, angle of Lanelet Line relative to UTM in rad
   */
  static double getLaneletLineHeading(const ConstLineString3d& laneletLine, const double& length) {
    return getLaneletLineHeading(utils::to2D(laneletLine).basicLineString(), length);
  }

  /**
   * @brief Returns the approximate heading of a 2D lanelet line at a given arc length.
   *
   * @param laneletLine line string to sample
   * @param length arc length along the line
   * @return heading angle relative to UTM in radians
   */
  static double getLaneletLineHeading(const ConstLineString2d& laneletLine, const double& length) {
    return getLaneletLineHeading(laneletLine.basicLineString(), length);
  }

  /**
   * @brief Returns the approximate heading of a basic 2D line at a given arc length.
   *
   * @param laneletLine line string to sample
   * @param length arc length along the line
   * @return heading angle relative to UTM in radians
   */
  static double getLaneletLineHeading(const BasicLineString2d& laneletLine, const double& length) {
    double total_length = 0;
    BasicPoint2d point_x_y_before = fromArcCoordinates_fast(laneletLine, std::max(0.0, length - 0.25), 0., total_length);
    if (total_length > 0) {
      point_x_y_before = fromArcCoordinates_fast(laneletLine, std::max(0.0, total_length - 0.25), 0.);
    }
    BasicPoint2d point_x_y_after = fromArcCoordinates_fast(laneletLine, length + 0.25, 0.);
    double lanelet_heading = atan2(point_x_y_after.y() - point_x_y_before.y(), point_x_y_after.x() - point_x_y_before.x());

    // Limit to 2PI
    while (lanelet_heading > M_PI * 2) {
      lanelet_heading -= M_PI * 2;
    }
    while (lanelet_heading < 0.) {
      lanelet_heading += M_PI * 2;
    }

    return lanelet_heading;
  }

  /**
   * @brief sorts a vector of lanelets according to their probability for being the correct matched lanelet
   * @param point, x/y point - BasicPoint2d
   * @param lanelets, vector of lanelets produced for example by geometry::findNearest() - std::vector<std::pair<double, Lanelet>>
   * @param heading(optional), optional heading of object or ego vehicle realtive to UTM in rad - double
   * @param trafficRules(optional), optional lanelet-traffic-rules-object which defines which lanelets are passable by which participant - traffic_rules::TrafficRulesPtr
   * @param last_lanelet_id(optional), optional last_lanelet_id for better matching over time - long int
   * @return nothing, but input vector of lanelets is sorted
   */
  static std::vector<double> laneletSorting(const BasicPoint2d& point,
                                            std::vector<std::pair<double, ConstLanelet>>& lanelets,
                                            boost::optional<const float&> heading,
                                            boost::optional<const traffic_rules::TrafficRulesPtr&> trafficRules,
                                            boost::optional<const int64_t&> last_lanelet_id) {
    // lanelets[i].first = distance to lanelet
    // lanelets[i].second = lanelet
    std::vector<double> costs;
    if (lanelets.empty()) {
      return costs;
    }

    struct lanelet_n_cost {
      std::pair<double, ConstLanelet> lanelet;
      double lanelet_cost;
    };

    std::vector<lanelet_n_cost> lanelets_n_costs;

    for (uint i = 0; i < lanelets.size(); i++) {
      const ArcCoordinates arccordinates_point = geometry::toArcCoordinates(lanelets[i].second.centerline2d(), point);
      const bool is_inside = lanelet::geometry::inside(lanelets[i].second, point);

      double cost_for_this_lanelet = (is_inside ? 0.5 : 2.0) * arccordinates_point.distance * arccordinates_point.distance;

      if (!!heading) {
        double diff_heading;  // rad

        diff_heading = std::abs(*heading - getLaneletLineHeading(lanelets[i].second.centerline(), arccordinates_point.length));

        if (diff_heading > M_PI) {
          diff_heading = M_PI * 2.0 - diff_heading;
        }

        if (!!trafficRules && !(*trafficRules)->isOneWay(lanelets[i].second) && diff_heading > M_PI / 2.0) {
          diff_heading = M_PI - diff_heading;
        }

        cost_for_this_lanelet += 10.0 * diff_heading * diff_heading;
      }

      if (!!trafficRules) {
        if (!(*trafficRules)->canPass(lanelets[i].second)) {
          cost_for_this_lanelet *= 2;
        }
      }

      if (!!last_lanelet_id) {
        if (*last_lanelet_id != lanelets[i].second.id()) {
          cost_for_this_lanelet *= 1.2;
        }
      }

      lanelets_n_costs.push_back({lanelets[i], cost_for_this_lanelet});
    }

    std::sort(lanelets_n_costs.begin(), lanelets_n_costs.end(),
              [](const auto& i, const auto& j) { return i.lanelet_cost < j.lanelet_cost; });

    for (uint i = 0; i < lanelets_n_costs.size(); i++) {
      lanelets.at(i) = lanelets_n_costs[i].lanelet;
      costs.push_back(lanelets_n_costs[i].lanelet_cost);
    }
    return costs;
  }

  /**
   * @brief Interpolates a point from arc coordinates on a 2D line string.
   *
   * @param line reference line string
   * @param length longitudinal arc length along the line
   * @param distance lateral offset from the line
   * @param line_length optional output for the total line length when `length` exceeds the line end
   * @return interpolated point in Cartesian coordinates
   */
  static BasicPoint2d fromArcCoordinates_fast(const BasicLineString2d& line,
                                              const double& length,
                                              const double& distance,
                                              boost::optional<double&> line_length = {}) {
    if (line.size() == 1) {
      return line.at(0);
    }

    double cur_len = 0.0;
    double cur_cum_len = 0.0;
    double remaining_dis = 0.0;
    size_t start_idx = 0;
    size_t end_idx = 0;
    bool is_s_bigger_length = false;

    for (size_t i = 0; i < line.size() - 1; i++) {
      cur_len = boost::geometry::distance(line.at(i), line.at(i + 1));
      cur_cum_len += cur_len;

      if (cur_cum_len > length) {
        remaining_dis = length - (cur_cum_len - cur_len);
        start_idx = i;
        end_idx = i + 1;
        break;
      }
    }

    // arc_length > line length
    if (end_idx == 0) {
      end_idx = line.size() - 1;
      start_idx = end_idx - 1;
      is_s_bigger_length = true;
      remaining_dis = boost::geometry::distance(line.at(start_idx), line.at(end_idx));

      if (!!line_length) {
        *line_length = cur_cum_len;
      }
    }

    // Very small line: generating point randomly around first point of line
    if (is_s_bigger_length && cur_cum_len < 0.05) {
      return line.at(0);
    }

    const double dx = cur_len == 0.0 ? 0 : (line.at(start_idx).x() - line.at(end_idx).x()) / cur_len;
    const double dy = cur_len == 0.0 ? 0 : (line.at(start_idx).y() - line.at(end_idx).y()) / cur_len;

    const double p_x = line.at(start_idx).x() - dx * remaining_dis;
    const double p_y = line.at(start_idx).y() - dy * remaining_dis;

    return BasicPoint2d(p_x + dy * distance, p_y - dx * distance);
  }

  /**
   * @brief Interpolates a point from arc coordinates on a const 2D line string.
   *
   * @param line reference line string
   * @param length longitudinal arc length along the line
   * @param distance lateral offset from the line
   * @return interpolated point in Cartesian coordinates
   */
  static BasicPoint2d fromArcCoordinates_fast(const ConstLineString2d& line, const double& length, const double& distance) {
    return fromArcCoordinates_fast(line.basicLineString(), length, distance);
  }

  /**
   * @brief Interpolates a point from arc coordinates on a const 3D line string.
   *
   * @param line reference line string
   * @param length longitudinal arc length along the line
   * @param distance lateral offset from the line
   * @return interpolated point in Cartesian coordinates
   */
  static BasicPoint2d fromArcCoordinates_fast(const ConstLineString3d& line, const double& length, const double& distance) {
    return fromArcCoordinates_fast(utils::to2D(line).basicLineString(), length, distance);
  }

  /**
   * @brief Interpolates a point from arc coordinates on a basic 2D line string.
   *
   * @param line reference line string
   * @param arc arc-coordinate pair
   * @return interpolated point in Cartesian coordinates
   */
  static BasicPoint2d fromArcCoordinates_fast(const BasicLineString2d& line, const ArcCoordinates& arc) {
    return fromArcCoordinates_fast(line, arc.length, arc.distance);
  }

  /**
   * @brief Interpolates a point from arc coordinates on a const 2D line string.
   *
   * @param line reference line string
   * @param arc arc-coordinate pair
   * @return interpolated point in Cartesian coordinates
   */
  static BasicPoint2d fromArcCoordinates_fast(const ConstLineString2d& line, const ArcCoordinates& arc) {
    return fromArcCoordinates_fast(line.basicLineString(), arc.length, arc.distance);
  }

  /**
   * @brief Interpolates a point from arc coordinates on a const 3D line string.
   *
   * @param line reference line string
   * @param arc arc-coordinate pair
   * @return interpolated point in Cartesian coordinates
   */
  static BasicPoint2d fromArcCoordinates_fast(const ConstLineString3d& line, const ArcCoordinates& arc) {
    return fromArcCoordinates_fast(utils::to2D(line).basicLineString(), arc.length, arc.distance);
  }
};
