// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <route_planning_msgs/msg/reference_line.hpp>
#include <std_msgs/msg/header.hpp>

namespace lanelet2_route_planning {

/**
 * @brief Encodes selected reference-line points as origin-relative float offsets.
 *
 * @param[in] header reference-line header
 * @param[in] line_string dense reference line
 * @param[in] retained_indices selected indices into line_string
 * @return compact reference-line message
 */
route_planning_msgs::msg::ReferenceLine createReferenceLineMessage(const std_msgs::msg::Header& header,
                                                                   const std::vector<Eigen::Vector2d>& line_string,
                                                                   const std::vector<size_t>& retained_indices);

}  // namespace lanelet2_route_planning
