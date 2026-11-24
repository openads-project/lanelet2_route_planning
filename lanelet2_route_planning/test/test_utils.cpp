// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include "lanelet2_route_planning/utils.hpp"

namespace lanelet2_route_planning {

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

/**
 * @brief Measures boundary positions counterclockwise for both ring orientations.
 */
TEST(BoundaryPositionTest, IsIndependentOfRingOrientation) {  // NOLINT
  const lanelet::BasicPolygon2d counterclockwise{{-10.0, -10.0}, {10.0, -10.0}, {10.0, 10.0}, {-10.0, 10.0}};
  const lanelet::BasicPolygon2d clockwise{{-10.0, -10.0}, {-10.0, 10.0}, {10.0, 10.0}, {10.0, -10.0}};

  for (const auto& ring : {counterclockwise, clockwise}) {
    EXPECT_DOUBLE_EQ(boundaryPosition(ring, lanelet::BasicPoint2d(0.0, -10.0)), 10.0);
    EXPECT_DOUBLE_EQ(boundaryPosition(ring, lanelet::BasicPoint2d(10.0, 0.0)), 30.0);
    EXPECT_DOUBLE_EQ(boundaryPosition(ring, lanelet::BasicPoint2d(0.0, 12.0)), 50.0);  // projected onto the ring
  }
}

/**
 * @brief Four-arm intersection in right-hand traffic with reciprocal left-turn rules on the east-west road.
 *
 * The square Area spans [-10, 10] in x and y. Eastbound lanes use y in [-3.5, 0], westbound lanes y in [0, 3.5],
 * the northern exit x in [0, 3.5], and the southern exit x in [-3.5, 0].
 */
class ReciprocalRightOfWayTest : public ::testing::Test {
 protected:
  /**
   * @brief Creates the lanelets and the intersection area; rules are added per test.
   */
  void SetUp() override {
    // eastbound approach and movements
    east_approach_ = makeLanelet({{-30.0, 0.0}, {-10.0, 0.0}}, {{-30.0, -3.5}, {-10.0, -3.5}});
    east_straight_ = makeLanelet({{-10.0, 0.0}, {10.0, 0.0}}, {{-10.0, -3.5}, {10.0, -3.5}});
    east_left_ = makeLanelet({{-10.0, 0.0}, {0.0, 10.0}}, {{-10.0, -3.5}, {3.5, 10.0}});
    east_right_ = makeLanelet({{-10.0, 0.0}, {0.0, -10.0}}, {{-10.0, -3.5}, {-3.5, -10.0}});
    east_exit_ = makeLanelet({{10.0, 0.0}, {30.0, 0.0}}, {{10.0, -3.5}, {30.0, -3.5}});

    // westbound approach and movements
    west_approach_ = makeLanelet({{30.0, 0.0}, {10.0, 0.0}}, {{30.0, 3.5}, {10.0, 3.5}});
    west_straight_ = makeLanelet({{10.0, 0.0}, {-10.0, 0.0}}, {{10.0, 3.5}, {-10.0, 3.5}});
    west_left_ = makeLanelet({{10.0, 0.0}, {0.0, -10.0}}, {{10.0, 3.5}, {-3.5, -10.0}});
    west_right_ = makeLanelet({{10.0, 0.0}, {0.0, 10.0}}, {{10.0, 3.5}, {3.5, 10.0}});
    west_exit_ = makeLanelet({{-10.0, 0.0}, {-30.0, 0.0}}, {{-10.0, 3.5}, {-30.0, 3.5}});

    // exits of the side arms
    north_exit_ = makeLanelet({{0.0, 10.0}, {0.0, 30.0}}, {{3.5, 10.0}, {3.5, 30.0}});
    south_exit_ = makeLanelet({{0.0, -10.0}, {0.0, -30.0}}, {{-3.5, -10.0}, {-3.5, -30.0}});

    // intersection area
    const auto outer = makeBound({{-10.0, -10.0}, {10.0, -10.0}, {10.0, 10.0}, {-10.0, 10.0}});
    area_ = lanelet::Area(next_id_++, {outer});
    area_.setAttribute(lanelet::AttributeName::Subtype, "intersection");
    ring_ = {{-10.0, -10.0}, {10.0, -10.0}, {10.0, 10.0}, {-10.0, 10.0}};
  }

  /**
   * @brief Adds a RightOfWay in which yield_lanelet yields to priority_lanelet within the intersection area.
   */
  lanelet::RightOfWay::Ptr addRightOfWay(lanelet::Lanelet& priority_lanelet, lanelet::Lanelet& yield_lanelet) {
    // use the factory like the map loader, since RightOfWay::make does not accept an intersection area
    lanelet::RuleParameterMap parameters;
    parameters["right_of_way"] = {lanelet::WeakLanelet(priority_lanelet)};
    parameters["yield"] = {lanelet::WeakLanelet(yield_lanelet)};
    parameters["intersection_area"] = {lanelet::WeakArea(area_)};
    auto right_of_way = std::dynamic_pointer_cast<lanelet::RightOfWay>(
        lanelet::RegulatoryElementFactory::create(lanelet::RightOfWay::RuleName, next_id_++, parameters));
    priority_lanelet.addRegulatoryElement(right_of_way);
    yield_lanelet.addRegulatoryElement(right_of_way);
    return right_of_way;
  }

  /**
   * @brief Builds map and vehicle routing graph after all rules have been added.
   */
  void buildRoutingGraph() {
    map_ = lanelet::utils::createMap({east_approach_, east_straight_, east_left_, east_right_, east_exit_, west_approach_,
                                      west_straight_, west_left_, west_right_, west_exit_, north_exit_, south_exit_},
                                     {area_});
    vehicle_.traffic_rules =
        lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    vehicle_.graph = lanelet::routing::RoutingGraph::build(*map_, *vehicle_.traffic_rules);
  }

  /**
   * @brief Evaluates keepYieldForRoute for a route starting at its yielding approach.
   */
  bool keepYield(const lanelet::ConstLanelets& route, const lanelet::RightOfWay& right_of_way) const {
    return keepYieldForRoute(lanelet::routing::LaneletPath(route), 0, right_of_way, {&vehicle_});
  }

  lanelet::Lanelet east_approach_, east_straight_, east_left_, east_right_, east_exit_;
  lanelet::Lanelet west_approach_, west_straight_, west_left_, west_right_, west_exit_;
  lanelet::Lanelet north_exit_, south_exit_;
  lanelet::Area area_;
  lanelet::BasicPolygon2d ring_;

 private:
  /**
   * @brief Returns the point at the given position, sharing points so that consecutive lanelets are connected.
   */
  lanelet::Point3d point(double x, double y) {
    const auto key = std::make_pair(x, y);
    const auto it = points_.find(key);
    if (it != points_.end()) {
      return it->second;
    }
    return points_.emplace(key, lanelet::Point3d(next_id_++, x, y, 0.0)).first->second;
  }

  /**
   * @brief Creates a bound from (x, y) coordinates.
   */
  lanelet::LineString3d makeBound(const std::vector<std::pair<double, double>>& coordinates) {
    lanelet::Points3d points;
    for (const auto& [x, y] : coordinates) {
      points.push_back(point(x, y));
    }
    return lanelet::LineString3d(next_id_++, points);
  }

  /**
   * @brief Creates an urban one-way road lanelet from (x, y) bound coordinates.
   */
  lanelet::Lanelet makeLanelet(const std::vector<std::pair<double, double>>& left,
                               const std::vector<std::pair<double, double>>& right) {
    const auto left_bound = makeBound(left);
    const auto right_bound = makeBound(right);
    lanelet::Lanelet lanelet(next_id_++, left_bound, right_bound);
    lanelet.setAttribute(lanelet::AttributeName::Subtype, lanelet::AttributeValueString::Road);
    lanelet.setAttribute(lanelet::AttributeName::Location, lanelet::AttributeValueString::Urban);
    lanelet.setAttribute(lanelet::AttributeName::OneWay, true);
    return lanelet;
  }

  lanelet::Id next_id_ = 1000;
  std::map<std::pair<double, double>, lanelet::Point3d> points_;
  lanelet::LaneletMapUPtr map_;
  ParticipantRoutingGraph vehicle_;
};

/**
 * @brief Classifies movements by their exit order relative to the oncoming entry, independent of turn angles.
 */
TEST_F(ReciprocalRightOfWayTest, ClassifiesMovementsAcrossOncomingTraffic) {  // NOLINT
  EXPECT_EQ(crossesOncoming(east_approach_, east_exit_, west_approach_, ring_), CrossesOncoming::NotLeft);
  EXPECT_EQ(crossesOncoming(east_approach_, south_exit_, west_approach_, ring_), CrossesOncoming::NotLeft);
  EXPECT_EQ(crossesOncoming(east_approach_, north_exit_, west_approach_, ring_), CrossesOncoming::Left);
  EXPECT_EQ(crossesOncoming(west_approach_, south_exit_, east_approach_, ring_), CrossesOncoming::Left);
  EXPECT_EQ(crossesOncoming(west_approach_, north_exit_, east_approach_, ring_), CrossesOncoming::NotLeft);

  // a U-turn crosses oncoming traffic
  EXPECT_EQ(crossesOncoming(east_approach_, west_exit_, west_approach_, ring_), CrossesOncoming::Left);

  // a skewed exit turning only 35 degrees to the left still crosses oncoming traffic
  const double heading = 35.0 * M_PI / 180.0;
  const lanelet::Point3d left_start(1, 6.5, 10.0, 0.0);
  const lanelet::Point3d right_start(2, 10.0, 5.0, 0.0);
  const lanelet::Point3d left_end(3, left_start.x() + 20.0 * std::cos(heading), left_start.y() + 20.0 * std::sin(heading), 0.0);
  const lanelet::Point3d right_end(4, right_start.x() + 20.0 * std::cos(heading), right_start.y() + 20.0 * std::sin(heading),
                                   0.0);
  const lanelet::Lanelet skewed_exit(5, lanelet::LineString3d(6, {left_start, left_end}),
                                     lanelet::LineString3d(7, {right_start, right_end}));
  EXPECT_EQ(crossesOncoming(east_approach_, skewed_exit, west_approach_, ring_), CrossesOncoming::Left);

  // exit and oncoming entry on a shared bidirectional lanelet cannot be ordered
  EXPECT_EQ(crossesOncoming(east_approach_, east_exit_, lanelet::ConstLanelet(east_exit_).invert(), ring_),
            CrossesOncoming::Ambiguous);
}

/**
 * @brief Detects a reciprocal RightOfWay only within the same intersection area.
 */
TEST_F(ReciprocalRightOfWayTest, DetectsReciprocalRightOfWay) {  // NOLINT
  addRightOfWay(west_approach_, east_approach_);
  EXPECT_FALSE(yieldsToApproach(west_approach_, east_approach_, area_.id()));

  addRightOfWay(east_approach_, west_approach_);
  EXPECT_TRUE(yieldsToApproach(west_approach_, east_approach_, area_.id()));
  EXPECT_FALSE(yieldsToApproach(west_approach_, east_approach_, area_.id() + 1));
}

/**
 * @brief Omits Yield for straight and right-turn routes if the conflicting oncoming left turn yields to the route.
 */
TEST_F(ReciprocalRightOfWayTest, OmitsYieldIfOnlyReciprocalLeftTurnsConflict) {  // NOLINT
  const auto east_yields = addRightOfWay(west_approach_, east_approach_);
  addRightOfWay(east_approach_, west_approach_);
  buildRoutingGraph();

  EXPECT_FALSE(keepYield({east_approach_, east_straight_, east_exit_}, *east_yields));
  EXPECT_FALSE(keepYield({east_approach_, east_right_, south_exit_}, *east_yields));
}

/**
 * @brief Keeps Yield for a left-turn route, which crosses oncoming traffic.
 */
TEST_F(ReciprocalRightOfWayTest, KeepsYieldForLeftTurnRoute) {  // NOLINT
  const auto east_yields = addRightOfWay(west_approach_, east_approach_);
  addRightOfWay(east_approach_, west_approach_);
  buildRoutingGraph();

  EXPECT_TRUE(keepYield({east_approach_, east_left_, north_exit_}, *east_yields));
}

/**
 * @brief Keeps Yield for a straight route if the oncoming left turn does not yield to the route.
 */
TEST_F(ReciprocalRightOfWayTest, KeepsYieldWithoutReciprocalRightOfWay) {  // NOLINT
  const auto east_yields = addRightOfWay(west_approach_, east_approach_);
  buildRoutingGraph();

  EXPECT_TRUE(keepYield({east_approach_, east_straight_, east_exit_}, *east_yields));
}

}  // namespace lanelet2_route_planning
