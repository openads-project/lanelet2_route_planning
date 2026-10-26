// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/Route.h>

#include "lanelet2_route_planning/utils.hpp"

namespace lanelet2_route_planning {

class RouteContextTest : public ::testing::Test {
 protected:
  void SetUp() override {
    lanelet::Points3d left_points;
    lanelet::Points3d right_points;
    for (int i = 0; i <= 4; ++i) {
      left_points.emplace_back(100 + i, 10.0 * i, 1.0, 0.0);
      right_points.emplace_back(200 + i, 10.0 * i, -1.0, 0.0);
    }
    for (size_t i = 0; i < 4; ++i) {
      lanelet::LineString3d left(300 + 2 * i, {left_points[i], left_points[i + 1]});
      lanelet::LineString3d right(301 + 2 * i, {right_points[i], right_points[i + 1]});
      lanelet::Lanelet lanelet(400 + i, left, right);
      lanelet.attributes()["subtype"] = "road";
      lanelet.attributes()["location"] = "urban";
      lanelet.attributes()["one_way"] = (i == 1 || i == 2) ? "no" : "yes";
      // Store the middle lanelets opposite to the route's eastbound travel direction.
      if (i == 1 || i == 2) {
        lanelet = lanelet.invert();
      }
      map_->add(lanelet);
      lanelets_.push_back(lanelet);
    }
    graph_ = lanelet::routing::RoutingGraph::build(*map_, *getTrafficRules());
  }

  lanelet::LaneletMapPtr map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::ConstLanelets lanelets_;
  lanelet::routing::RoutingGraphUPtr graph_;
};

/**
 * @brief Extends a bidirectional destination after choosing its reachable direction.
 */
TEST_F(RouteContextTest, OvershootsInSelectedDestinationDirection) {  // NOLINT
  const auto wrong_destination = followLaneletsAlongRoutingGraph(graph_, lanelets_[2], {25.0, 0.0}, 10.0);
  EXPECT_EQ(wrong_destination, lanelets_[1]);
  const auto route = getRouteWithContext(graph_, {lanelets_[0], lanelets_[2]}, {5.0, 0.0}, {25.0, 0.0}, 0.0, 10.0);

  ASSERT_TRUE(route);
  const auto path = route->shortestPath();
  ASSERT_EQ(path.size(), 4U);
  EXPECT_EQ(path[2], lanelets_[2].invert());
  EXPECT_EQ(path.back(), lanelets_[3]);
}

/**
 * @brief Reproduces rejection when map-oriented overshoot follows an unreachable outgoing branch.
 */
TEST_F(RouteContextTest, AvoidsUnreachableBranchBeyondMapOrientedDestination) {  // NOLINT
  const auto destination = lanelets_[1];
  lanelet::LineString3d left(500,
                             {map_->pointLayer.get(destination.leftBound().back().id()), lanelet::Point3d(501, 0.0, 4.0, 0.0)});
  lanelet::LineString3d right(502,
                              {map_->pointLayer.get(destination.rightBound().back().id()), lanelet::Point3d(503, 0.0, 6.0, 0.0)});
  lanelet::Lanelet branch(504, left, right);
  branch.attributes()["subtype"] = "road";
  branch.attributes()["location"] = "urban";
  branch.attributes()["one_way"] = "yes";
  map_->add(branch);
  graph_ = lanelet::routing::RoutingGraph::build(*map_, *getTrafficRules());

  const auto wrong_destination = followLaneletsAlongRoutingGraph(graph_, destination, {15.0, 0.0}, 10.0);
  ASSERT_EQ(wrong_destination, branch);
  EXPECT_FALSE(getRoute(graph_, {lanelets_[0], wrong_destination}));

  const auto route = getRouteWithContext(graph_, {lanelets_[0], destination}, {5.0, 0.0}, {15.0, 0.0}, 0.0, 10.0);
  ASSERT_TRUE(route);
  const auto path = route->shortestPath();
  ASSERT_EQ(path.size(), 3U);
  EXPECT_EQ(path[1], destination.invert());
  EXPECT_EQ(path.back(), lanelets_[2].invert());
}

/**
 * @brief Extends a bidirectional start behind the selected travel direction.
 */
TEST_F(RouteContextTest, UndershootsInSelectedStartDirection) {  // NOLINT
  const auto route = getRouteWithContext(graph_, {lanelets_[1], lanelets_[3]}, {15.0, 0.0}, {35.0, 0.0}, 10.0, 0.0);

  ASSERT_TRUE(route);
  const auto path = route->shortestPath();
  ASSERT_EQ(path.size(), 4U);
  EXPECT_EQ(path.front(), lanelets_[0]);
  EXPECT_EQ(path[1], lanelets_[1].invert());
}

/**
 * @brief Keeps both requested endpoints when extending both ends of a route.
 */
TEST_F(RouteContextTest, ExtendsBothEndpoints) {  // NOLINT
  const auto route = getRouteWithContext(graph_, {lanelets_[1], lanelets_[2]}, {15.0, 0.0}, {25.0, 0.0}, 10.0, 10.0);

  ASSERT_TRUE(route);
  const auto path = route->shortestPath();
  ASSERT_EQ(path.size(), 4U);
  EXPECT_EQ(path.front(), lanelets_[0]);
  EXPECT_EQ(path[1], lanelets_[1].invert());
  EXPECT_EQ(path[2], lanelets_[2].invert());
  EXPECT_EQ(path.back(), lanelets_[3]);
}

/**
 * @brief Zero extension still selects the reachable bidirectional endpoint.
 */
TEST_F(RouteContextTest, SupportsZeroContext) {  // NOLINT
  const auto route = getRouteWithContext(graph_, {lanelets_[0], lanelets_[2]}, {5.0, 0.0}, {25.0, 0.0}, 0.0, 0.0);

  ASSERT_TRUE(route);
  EXPECT_EQ(route->shortestPath().size(), 3U);
  EXPECT_EQ(route->shortestPath().back(), lanelets_[2].invert());
}

/**
 * @brief Context stops at graph boundaries without rejecting a valid route.
 */
TEST_F(RouteContextTest, StopsContextAtMapBoundary) {  // NOLINT
  const auto route = getRouteWithContext(graph_, {lanelets_[0], lanelets_[3]}, {5.0, 0.0}, {35.0, 0.0}, 100.0, 100.0);

  ASSERT_TRUE(route);
  EXPECT_EQ(route->shortestPath().size(), 4U);
}

/**
 * @brief Requested intermediate lanelets remain on the extended route.
 */
TEST_F(RouteContextTest, PreservesIntermediateDestination) {  // NOLINT
  const auto route =
      getRouteWithContext(graph_, {lanelets_[0], lanelets_[1].invert(), lanelets_[2]}, {5.0, 0.0}, {25.0, 0.0}, 0.0, 10.0);

  ASSERT_TRUE(route);
  const auto path = route->shortestPath();
  ASSERT_EQ(path.size(), 4U);
  EXPECT_EQ(path[1], lanelets_[1].invert());
  EXPECT_EQ(path.back(), lanelets_[3]);
}

/**
 * @brief Rejects a route that would require reversing a one-way lanelet.
 */
TEST_F(RouteContextTest, DoesNotReverseOneWayLanelets) {  // NOLINT
  EXPECT_FALSE(getRouteWithContext(graph_, {lanelets_[3], lanelets_[0]}, {35.0, 0.0}, {5.0, 0.0}, 10.0, 10.0));
}

/**
 * @brief Incomplete endpoint input has no route.
 */
TEST_F(RouteContextTest, RejectsMissingEndpoints) {  // NOLINT
  EXPECT_FALSE(getRoute(graph_, {}));
  EXPECT_FALSE(getRoute(graph_, {lanelets_[0]}));
}

class ExtractRegulatoryElementsTest : public ::testing::Test {
 protected:
  /**
   * @brief Creates a straight lanelet centered at the given lateral position.
   */
  static lanelet::Lanelet makeLanelet(lanelet::Id id, double y) {
    lanelet::LineString3d left_bound(id + 1,
                                     {lanelet::Point3d(id + 2, 0.0, y + 1.0, 0.0), lanelet::Point3d(id + 3, 10.0, y + 1.0, 0.0)});
    lanelet::LineString3d right_bound(
        id + 4, {lanelet::Point3d(id + 5, 0.0, y - 1.0, 0.0), lanelet::Point3d(id + 6, 10.0, y - 1.0, 0.0)});
    return lanelet::Lanelet(id, left_bound, right_bound);
  }

  /**
   * @brief Creates a two-point line string.
   */
  static lanelet::LineString3d makeLine(lanelet::Id id, double x1, double y1, double x2, double y2) {
    return lanelet::LineString3d(id, {lanelet::Point3d(id + 1, x1, y1, 0.0), lanelet::Point3d(id + 2, x2, y2, 0.0)});
  }

  /**
   * @brief Creates a straight three-point route sequence.
   */
  static PointSequence makePointSequence(double y, double current_x = 7.0, double next_x = 9.0) {
    return PointSequence(Eigen::Vector2d(current_x - 2.0, y), Eigen::Vector2d(current_x, y), Eigen::Vector2d(next_x, y));
  }
};

/**
 * @brief Assigns a right-of-way rule only to its yielding adjacent lanelet.
 */
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

/**
 * @brief Assigns an adjacent-lane rule when only its own centerline intersects the stop line.
 */
TEST_F(ExtractRegulatoryElementsTest, AssignsAdjacentLaneRuleUsingOwnCenterline) {  // NOLINT
  auto priority_lanelet = makeLanelet(100, 4.0);
  auto yield_lanelet = makeLanelet(200, -4.0);
  auto stop_line = makeLine(300, 8.0, -5.0, 8.0, -3.0);
  auto right_of_way = lanelet::RightOfWay::make(400, lanelet::AttributeMap(), {priority_lanelet}, {yield_lanelet}, stop_line);
  yield_lanelet.addRegulatoryElement(right_of_way);

  const auto result = extractRegulatoryElements(priority_lanelet, {}, {yield_lanelet}, makePointSequence(0.0));

  ASSERT_EQ(result.regulatory_element_msgs.size(), 1U);
  EXPECT_TRUE(result.regulatory_element_idcs.empty());
  ASSERT_EQ(result.adjacent_right_regulatory_element_idcs.size(), 1U);
  EXPECT_EQ(result.adjacent_right_regulatory_element_idcs.front(), std::vector<uint8_t>({0U}));
}

/**
 * @brief Omits a stop line that does not intersect either local route segment.
 */
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

/**
 * @brief Assigns an adjacent stop line on a shared endpoint only to the preceding route element.
 */
TEST_F(ExtractRegulatoryElementsTest, AssignsStopLineToPrecedingRouteElement) {  // NOLINT
  auto priority_lanelet = makeLanelet(100, 4.0);
  auto yield_lanelet = makeLanelet(200, 0.0);
  auto stop_line = makeLine(300, 8.0, -1.0, 8.0, 1.0);
  auto right_of_way = lanelet::RightOfWay::make(400, lanelet::AttributeMap(), {priority_lanelet}, {yield_lanelet}, stop_line);
  yield_lanelet.addRegulatoryElement(right_of_way);

  const auto preceding_result =
      extractRegulatoryElements(priority_lanelet, {}, {yield_lanelet}, makePointSequence(4.0, 6.0, 8.0));
  const auto following_result =
      extractRegulatoryElements(priority_lanelet, {}, {yield_lanelet},
                                PointSequence(Eigen::Vector2d(6.0, 4.0), Eigen::Vector2d(8.0, 4.0), Eigen::Vector2d(10.0, 4.0)));

  ASSERT_EQ(preceding_result.regulatory_element_msgs.size(), 1U);
  EXPECT_TRUE(following_result.regulatory_element_msgs.empty());
}

/**
 * @brief Uses the yielding lanelet end when no explicit stop line exists.
 */
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

/**
 * @brief Keeps all-way-stop lines separate when they differ per lanelet.
 */
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

/**
 * @brief Uses the lanelet end for an all-way stop with no explicit line.
 */
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

/**
 * @brief Uses the lanelet end for a traffic light with no explicit stop line.
 */
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
