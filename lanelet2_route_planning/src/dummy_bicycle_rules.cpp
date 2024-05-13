#include "lanelet2_route_planning/dummy_bicycle_rules.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"

namespace lanelet {
namespace traffic_rules {

RegisterTrafficRules<DummyBicycle> dbRules(
    std::string(Locations::Germany) + ":dummy",
    Participants::Bicycle);  // Lanelet does not allow to subclass bicycle easily; so subclass the country instead

LaneChangeType DummyBicycle::laneChangeType(const ConstLineString3d& boundary, bool virtualIsPassable = false) const {
  return GermanBicycle::laneChangeType(boundary, true);
}

}  // namespace traffic_rules
}  // namespace lanelet
