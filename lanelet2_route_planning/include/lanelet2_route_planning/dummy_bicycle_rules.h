#pragma once
#include <lanelet2_core/utility/Units.h>
#include "lanelet2_traffic_rules/GermanTrafficRules.h"

namespace lanelet {
namespace traffic_rules {

class DummyBicycle : public GermanBicycle {
 public:
  using GermanBicycle::GermanBicycle;

 protected:
  virtual LaneChangeType laneChangeType(const ConstLineString3d& boundary, bool virtualIsPassable) const;
};
}  // namespace traffic_rules
}  // namespace lanelet
