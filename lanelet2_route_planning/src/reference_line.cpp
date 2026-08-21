// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include "lanelet2_route_planning/reference_line.hpp"

namespace lanelet2_route_planning {

lanelet2_route_planning_msgs::msg::ReferenceLine createReferenceLineMessage(const std_msgs::msg::Header& header,
                                                                            const std::vector<Eigen::Vector2d>& line_string,
                                                                            const std::vector<size_t>& retained_indices) {
  lanelet2_route_planning_msgs::msg::ReferenceLine message;
  message.header = header;
  if (retained_indices.empty()) {
    return message;
  }

  const Eigen::Vector2d& origin = line_string.at(retained_indices.front());
  message.origin.x = origin.x();
  message.origin.y = origin.y();
  message.delta_points.reserve(retained_indices.size() - 1);
  for (auto retained_idx_it = retained_indices.begin() + 1; retained_idx_it != retained_indices.end(); ++retained_idx_it) {
    const Eigen::Vector2d delta = line_string.at(*retained_idx_it) - origin;
    lanelet2_route_planning_msgs::msg::Point2D32 delta_point;
    delta_point.x = static_cast<float>(delta.x());
    delta_point.y = static_cast<float>(delta.y());
    message.delta_points.push_back(delta_point);
  }
  return message;
}

}  // namespace lanelet2_route_planning
