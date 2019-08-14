/*
   Waypoint yaml parser

   LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE

   Author : Jihoon Lee
   Date   : Dec 2013
 */

#ifndef WAYPOINT_MANAGER_YAML_PARSER_H
#define WAYPOINT_MANAGER_YAML_PARSER_H

#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include "TrajectoryList.h"
#include "WaypointList.h"

class yaml_parser {
  public:


  bool loadWaypointsAndTrajectoriesFromYaml(const std::string& filename,
                                            waypoint_msgs::WaypointList& wps,
                                            waypoint_msgs::TrajectoryList& trajs);
  void getYamlNode(const std::string& filename, YAML::Node& node);
  void parseWaypoints(const YAML::Node& node, waypoint_msgs::WaypointList& wps);
  void parseTrajectories(const YAML::Node& node, const waypoint_msgs::WaypointList& wps, waypoint_msgs::TrajectoryList& trajs);
  private:
};


// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}


#endif
