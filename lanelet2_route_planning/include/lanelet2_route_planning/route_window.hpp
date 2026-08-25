// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>

#include <route_planning_msgs/msg/route.hpp>

namespace lanelet2_route_planning {

/**
 * @brief A local route window and its offset in the complete route.
 */
struct LocalRouteWindow {
  route_planning_msgs::msg::Route route;
  size_t first_global_idx = 0;
};

/**
 * @brief Extracts a local route window while preserving absolute route data.
 *
 * Start and destination indices are remapped when present and set to
 * Route::INVALID_ROUTE_ELEMENT_IDX otherwise. The current index is always local and valid for a valid input.
 *
 * @param[in] full_route complete route
 * @param[in] current_global_idx current element index in full_route
 * @param[in] distance_behind distance to include behind the current element [m]
 * @param[in] distance_ahead distance to include ahead of the current element [m]
 * @return extracted local route and its global offset
 */
inline LocalRouteWindow extractLocalRouteWindow(const route_planning_msgs::msg::Route& full_route,
                                                const size_t current_global_idx,
                                                const double distance_behind,
                                                const double distance_ahead) {
  LocalRouteWindow result;
  result.route = full_route;
  const auto& full_route_elements = full_route.route_elements;
  if (current_global_idx >= full_route_elements.size()) {
    result.route.route_elements.clear();
    return result;
  }

  const double current_s = full_route_elements[current_global_idx].s;
  const auto first_it = std::lower_bound(
      full_route_elements.begin(), full_route_elements.end(), current_s - distance_behind,
      [](const route_planning_msgs::msg::RouteElement& route_element, const double s) { return route_element.s < s; });
  const auto last_it = std::upper_bound(
      full_route_elements.begin(), full_route_elements.end(), current_s + distance_ahead,
      [](const double s, const route_planning_msgs::msg::RouteElement& route_element) { return s < route_element.s; });
  result.first_global_idx = std::distance(full_route_elements.begin(), first_it);
  const size_t last_global_idx = std::distance(full_route_elements.begin(), last_it);
  result.route.route_elements.assign(first_it, last_it);

  const auto remap_global_idx = [first_global_idx = result.first_global_idx, last_global_idx](const uint64_t global_idx) {
    return (global_idx >= first_global_idx && global_idx < last_global_idx)
               ? global_idx - first_global_idx
               : route_planning_msgs::msg::Route::INVALID_ROUTE_ELEMENT_IDX;
  };
  result.route.starting_route_element_idx = remap_global_idx(full_route.starting_route_element_idx);
  result.route.current_route_element_idx = current_global_idx - result.first_global_idx;
  result.route.destination_route_element_idx = remap_global_idx(full_route.destination_route_element_idx);
  return result;
}

}  // namespace lanelet2_route_planning
