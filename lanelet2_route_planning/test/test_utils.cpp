// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include "lanelet2_route_planning/utils.hpp"

namespace lanelet2_route_planning {

class ExtractRegulatoryElementsTest : public ::testing::Test {
 protected:
  static lanelet::Lanelet makeLanelet(lanelet::Id id, double y) {
    lanelet::LineString3d left_bound(id + 1,
                                     {lanelet::Point3d(id + 2, 0.0, y + 1.0, 0.0), lanelet::Point3d(id + 3, 10.0, y + 1.0, 0.0)});
    lanelet::LineString3d right_bound(
        id + 4, {lanelet::Point3d(id + 5, 0.0, y - 1.0, 0.0), lanelet::Point3d(id + 6, 10.0, y - 1.0, 0.0)});
    return lanelet::Lanelet(id, left_bound, right_bound);
  }

  static lanelet::LineString3d makeLine(lanelet::Id id, double x1, double y1, double x2, double y2) {
    return lanelet::LineString3d(id, {lanelet::Point3d(id + 1, x1, y1, 0.0), lanelet::Point3d(id + 2, x2, y2, 0.0)});
  }

  static PointSequence makePointSequence(double y, double current_x = 7.0, double next_x = 9.0) {
    return PointSequence(Eigen::Vector2d(current_x - 2.0, y), Eigen::Vector2d(current_x, y), Eigen::Vector2d(next_x, y));
  }
};

TEST_F(ExtractRegulatoryElementsTest, AssignsRightOfWayRuleOnlyToYieldLanelet) {  // NOLINT
  auto priority_lanelet = makeLanelet(100, 0.0);
  auto yield_lanelet = makeLanelet(200, -2.0);
  auto stop_line = makeLine(300, 8.0, -3.0, 8.0, 1.0);
  auto right_of_way = lanelet::RightOfWay::make(400, lanelet::AttributeMap(), {priority_lanelet}, {yield_lanelet}, stop_line);
  priority_lanelet.addRegulatoryElement(right_of_way);
  yield_lanelet.addRegulatoryElement(right_of_way);

  const auto result = extractRegulatoryElements(priority_lanelet, {}, {yield_lanelet}, makePointSequence(0.0));

  ASSERT_EQ(result.regulatory_element_msgs.size(), 1U);
  EXPECT_TRUE(result.regulatory_element_idcs.empty());
  ASSERT_EQ(result.adjacent_right_regulatory_element_idcs.size(), 1U);
  EXPECT_EQ(result.adjacent_right_regulatory_element_idcs.front(), std::vector<uint8_t>({0U}));
  EXPECT_EQ(result.regulatory_element_msgs.front().type, route_planning_msgs::msg::RegulatoryElement::TYPE_YIELD);
}

TEST_F(ExtractRegulatoryElementsTest, IgnoresStopLineOutsideRouteSegment) {  // NOLINT
  auto priority_lanelet = makeLanelet(100, 4.0);
  auto yield_lanelet = makeLanelet(200, 0.0);
  auto distant_stop_line = makeLine(300, 8.0, 10.0, 8.0, 12.0);
  auto right_of_way =
      lanelet::RightOfWay::make(400, lanelet::AttributeMap(), {priority_lanelet}, {yield_lanelet}, distant_stop_line);
  yield_lanelet.addRegulatoryElement(right_of_way);

  const auto result = extractRegulatoryElements(yield_lanelet, {}, {}, makePointSequence(0.0));

  EXPECT_TRUE(result.regulatory_element_msgs.empty());
}

TEST_F(ExtractRegulatoryElementsTest, UsesLaneletEndForYieldWithoutStopLine) {  // NOLINT
  auto priority_lanelet = makeLanelet(100, 4.0);
  auto yield_lanelet = makeLanelet(200, 0.0);
  auto right_of_way = lanelet::RightOfWay::make(400, lanelet::AttributeMap(), {priority_lanelet}, {yield_lanelet});
  yield_lanelet.addRegulatoryElement(right_of_way);

  const auto result = extractRegulatoryElements(yield_lanelet, {}, {}, makePointSequence(0.0, 9.0, 11.0));

  ASSERT_EQ(result.regulatory_element_msgs.size(), 1U);
  EXPECT_EQ(result.regulatory_element_msgs.front().type, route_planning_msgs::msg::RegulatoryElement::TYPE_YIELD);
  const auto& line = result.regulatory_element_msgs.front().reference_line;
  EXPECT_DOUBLE_EQ(line[0].x, 10.0);
  EXPECT_DOUBLE_EQ(line[1].x, 10.0);
  EXPECT_DOUBLE_EQ(line[0].y, 1.0);
  EXPECT_DOUBLE_EQ(line[1].y, -1.0);
}

TEST_F(ExtractRegulatoryElementsTest, UsesLaneletSpecificAllWayStopLines) {  // NOLINT
  auto first_lanelet = makeLanelet(100, 0.0);
  auto second_lanelet = makeLanelet(200, -2.0);
  auto first_stop_line = makeLine(300, 8.0, -1.0, 8.0, 1.0);
  auto second_stop_line = makeLine(400, 9.0, -3.0, 9.0, 1.0);
  auto all_way_stop = lanelet::AllWayStop::make(500, lanelet::AttributeMap(),
                                                {{first_lanelet, first_stop_line}, {second_lanelet, second_stop_line}});
  first_lanelet.addRegulatoryElement(all_way_stop);
  second_lanelet.addRegulatoryElement(all_way_stop);

  const auto result = extractRegulatoryElements(first_lanelet, {}, {second_lanelet}, makePointSequence(0.0));

  ASSERT_EQ(result.regulatory_element_msgs.size(), 2U);
  EXPECT_EQ(result.regulatory_element_idcs, std::vector<uint8_t>({0U}));
  ASSERT_EQ(result.adjacent_right_regulatory_element_idcs.size(), 1U);
  EXPECT_EQ(result.adjacent_right_regulatory_element_idcs.front(), std::vector<uint8_t>({1U}));
  EXPECT_DOUBLE_EQ(result.regulatory_element_msgs[0].reference_line[0].x, 8.0);
  EXPECT_DOUBLE_EQ(result.regulatory_element_msgs[1].reference_line[0].x, 9.0);
}

TEST_F(ExtractRegulatoryElementsTest, UsesLaneletEndForAllWayStopWithoutStopLines) {  // NOLINT
  auto lanelet = makeLanelet(100, 0.0);
  auto all_way_stop = lanelet::AllWayStop::make(200, lanelet::AttributeMap(), {{lanelet, {}}});
  lanelet.addRegulatoryElement(all_way_stop);

  const auto result = extractRegulatoryElements(lanelet, {}, {}, makePointSequence(0.0, 9.0, 11.0));

  ASSERT_EQ(result.regulatory_element_msgs.size(), 1U);
  EXPECT_EQ(result.regulatory_element_msgs.front().type, route_planning_msgs::msg::RegulatoryElement::TYPE_STOP);
  EXPECT_DOUBLE_EQ(result.regulatory_element_msgs.front().reference_line[0].x, 10.0);
  EXPECT_DOUBLE_EQ(result.regulatory_element_msgs.front().reference_line[1].x, 10.0);
}

TEST_F(ExtractRegulatoryElementsTest, UsesLaneletEndForTrafficLightWithoutStopLine) {  // NOLINT
  auto lanelet = makeLanelet(100, 0.0);
  auto light_line = makeLine(200, 10.0, 2.0, 10.0, 3.0);
  auto traffic_light = lanelet::TrafficLight::make(300, lanelet::AttributeMap(), {light_line});
  lanelet.addRegulatoryElement(traffic_light);

  const auto result = extractRegulatoryElements(lanelet, {}, {}, makePointSequence(0.0, 9.0, 11.0));

  ASSERT_EQ(result.regulatory_element_msgs.size(), 1U);
  EXPECT_EQ(result.regulatory_element_msgs.front().type, route_planning_msgs::msg::RegulatoryElement::TYPE_TRAFFIC_LIGHT);
  EXPECT_DOUBLE_EQ(result.regulatory_element_msgs.front().reference_line[0].x, 10.0);
  EXPECT_DOUBLE_EQ(result.regulatory_element_msgs.front().reference_line[1].x, 10.0);
}

}  // namespace lanelet2_route_planning
