#pragma once

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <lanelet2_route_planning_interfaces/msg/route.hpp>
#include <lanelet2_route_planning_interfaces/msg/driveable_space.hpp>

namespace tf2 {
  
  using namespace lanelet2_route_planning_interfaces::msg;

  // Route
  inline void doTransform(const Route& route_in, Route& route_out, geometry_msgs::msg::TransformStamped& transform) {

    route_out = route_in;
    route_out.header.stamp = transform.header.stamp;
    route_out.header.frame_id = transform.header.frame_id;

    // target_position
    doTransform(route_in.target_position, route_out.target_position, transform);

    // boundaries
    // left
    for(int i=0; i<route_in.boundaries.left.size(); i++)
    {
        doTransform(route_in.boundaries.left[i], route_out.boundaries.left[i], transform);
    }
    // right
    for(int i=0; i<route_in.boundaries.right.size(); i++)
    {
        doTransform(route_in.boundaries.right[i], route_out.boundaries.right[i], transform);
    }

    // shortest_path
    for(int i=0; i<route_in.shortest_path.size(); i++)
    {
        doTransform(route_in.shortest_path[i], route_out.shortest_path[i], transform);
    }

    // road_markings
    for(int i=0; i<route_in.road_markings.size(); i++)
    {
        for(int j=0; j<route_in.road_markings[i].line.size(); j++)
        {
            doTransform(route_in.road_markings[i].line[j], route_out.road_markings[i].line[j], transform);
        }
    }

    //regulatory_elements
    for(int i=0; i<route_in.regulatory_elements.size(); i++)
    {
        for(int j=0; j<route_in.regulatory_elements[i].effect_line.size(); j++)
        {
            doTransform(route_in.regulatory_elements[i].effect_line[j], route_out.regulatory_elements[i].effect_line[j], transform);
        }
        doTransform(route_in.regulatory_elements[i].signal_position, route_out.regulatory_elements[i].signal_position, transform);
    }
  }

  inline const tf2::TimePoint& getTimestamp(const Route& route) {
    return tf2_ros::fromMsg(route.header.stamp);
  }

  inline const std::string& getFrameId(const Route& route) {
    return route.header.frame_id;
  }

  // DriveableSpace
  inline void doTransform(const DriveableSpace& ds_in, DriveableSpace& ds_out, geometry_msgs::msg::TransformStamped& transform) {

    ds_out = ds_in;
    ds_out.header.stamp = transform.header.stamp;
    ds_out.header.frame_id = transform.header.frame_id;

    // boundariess
    // left
    for(int i=0; i<ds_in.boundaries.left.size(); i++)
    {
        doTransform(ds_in.boundaries.left[i], ds_out.boundaries.left[i], transform);
    }
    // right
    for(int i=0; i<ds_in.boundaries.right.size(); i++)
    {
        doTransform(ds_in.boundaries.right[i], ds_out.boundaries.right[i], transform);
    }

    // restricted_areas
    for(int i=0; i<ds_in.restricted_areas.size(); i++)
    {
      for(int j=0; j<ds_in.restricted_areas[i].points.size(); j++)
      {
        geometry_msgs::msg::Point p_in, p_out;
        p_in.x=ds_in.restricted_areas[i].points[j].x;
        p_in.y=ds_in.restricted_areas[i].points[j].y;
        p_in.z=ds_in.restricted_areas[i].points[j].z;
        doTransform(p_in, p_out, transform);
        ds_out.restricted_areas[i].points[j].x = (float)p_out.x;
        ds_out.restricted_areas[i].points[j].y = (float)p_out.y;
        ds_out.restricted_areas[i].points[j].z = (float)p_out.z;
      }
    }
  }

  inline const tf2::TimePoint& getTimestamp(const DriveableSpace& ds) {
    return tf2_ros::fromMsg(ds.header.stamp);
  }

  inline const std::string& getFrameId(const DriveableSpace& ds) {
    return ds.header.frame_id;
  }

}
