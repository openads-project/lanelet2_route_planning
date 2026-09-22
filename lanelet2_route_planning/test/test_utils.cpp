// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include "lanelet2_route_planning/utils.hpp"

namespace lanelet2_route_planning {

TEST(ExtractRegulatoryElements, AssignsRightOfWayRuleOnlyToYieldLanelet) {  // NOLINT
  lanelet::Lanelet priority_lanelet(
      100, lanelet::LineString3d(101, {lanelet::Point3d(102, 0.0, 1.0, 0.0), lanelet::Point3d(103, 10.0, 1.0, 0.0)}),
      lanelet::LineString3d(104, {lanelet::Point3d(105, 0.0, -1.0, 0.0), lanelet::Point3d(106, 10.0, -1.0, 0.0)}));
  lanelet::Lanelet yield_lanelet(
      200, lanelet::LineString3d(201, {lanelet::Point3d(202, 0.0, -1.0, 0.0), lanelet::Point3d(203, 10.0, -1.0, 0.0)}),
      lanelet::LineString3d(204, {lanelet::Point3d(205, 0.0, -3.0, 0.0), lanelet::Point3d(206, 10.0, -3.0, 0.0)}));
  lanelet::LineString3d stop_line(300, {lanelet::Point3d(301, 8.0, -3.0, 0.0), lanelet::Point3d(302, 8.0, 1.0, 0.0)});
  auto right_of_way = lanelet::RightOfWay::make(400, lanelet::AttributeMap(), {priority_lanelet}, {yield_lanelet}, stop_line);
  priority_lanelet.addRegulatoryElement(right_of_way);
  yield_lanelet.addRegulatoryElement(right_of_way);

  const PointSequence point_sequence(Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(7.0, 0.0), Eigen::Vector2d(9.0, 0.0));
  const auto result = extractRegulatoryElements(priority_lanelet, {}, {yield_lanelet}, point_sequence);

  ASSERT_EQ(result.regulatory_element_msgs.size(), 1U);
  EXPECT_TRUE(result.regulatory_element_idcs.empty());
  ASSERT_EQ(result.adjacent_right_regulatory_element_idcs.size(), 1U);
  EXPECT_EQ(result.adjacent_right_regulatory_element_idcs.front(), std::vector<uint8_t>({0U}));
  EXPECT_EQ(result.regulatory_element_msgs.front().type, route_planning_msgs::msg::RegulatoryElement::TYPE_YIELD);
}

TEST(ExtractRegulatoryElements, IgnoresStopLineOutsideRouteSegment) {  // NOLINT
  lanelet::Lanelet priority_lanelet(
      100, lanelet::LineString3d(101, {lanelet::Point3d(102, 0.0, 5.0, 0.0), lanelet::Point3d(103, 10.0, 5.0, 0.0)}),
      lanelet::LineString3d(104, {lanelet::Point3d(105, 0.0, 3.0, 0.0), lanelet::Point3d(106, 10.0, 3.0, 0.0)}));
  lanelet::Lanelet yield_lanelet(
      200, lanelet::LineString3d(201, {lanelet::Point3d(202, 0.0, 1.0, 0.0), lanelet::Point3d(203, 10.0, 1.0, 0.0)}),
      lanelet::LineString3d(204, {lanelet::Point3d(205, 0.0, -1.0, 0.0), lanelet::Point3d(206, 10.0, -1.0, 0.0)}));
  lanelet::LineString3d distant_stop_line(300, {lanelet::Point3d(301, 8.0, 10.0, 0.0), lanelet::Point3d(302, 8.0, 12.0, 0.0)});
  auto right_of_way =
      lanelet::RightOfWay::make(400, lanelet::AttributeMap(), {priority_lanelet}, {yield_lanelet}, distant_stop_line);
  yield_lanelet.addRegulatoryElement(right_of_way);

  const PointSequence point_sequence(Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(7.0, 0.0), Eigen::Vector2d(9.0, 0.0));
  const auto result = extractRegulatoryElements(yield_lanelet, {}, {}, point_sequence);

  EXPECT_TRUE(result.regulatory_element_msgs.empty());
}

}  // namespace lanelet2_route_planning
