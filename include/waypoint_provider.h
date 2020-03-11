/*
   Way point Provider

   highly inspired by yocs_waypoint_navi written by Jorge Santos

   LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE

   Author : Jihoon Lee
   Date   : Dec 2013
 */


#ifndef WAYPOINT_PROVIDER_H
#define WAYPOINT_PROVIDER_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "Trajectory.h"
#include "TrajectoryList.h"
#include "WaypointListService.h"
#include "TrajectoryService.h"
#include "WaypointList.h"
#include <fstream>
#include "ros/ros.h"
#include <yaml-cpp/yaml.h>

/*
   It simply parses waypoints in yaml file and provides latch topic

   Future improvements
    - Utilise database for persistent waypoint storing
    - add/remove waypoints via srv or pub/sub
 */


  class WaypointProvider {
    public:
      WaypointProvider(ros::NodeHandle* n);
      ~WaypointProvider();
      bool loadWaypointsAndTrajectoriesFromYaml(const std::string& filename);


      void spin();
    protected:
      void init();
      bool processWaypointsService(waypoint_msgs::WaypointListService::Request& request,
                                   waypoint_msgs::WaypointListService::Response& response);
      bool processTrajectoryService(waypoint_msgs::TrajectoryService::Request& request,
                                   waypoint_msgs::TrajectoryService::Response& respond  );                                   
      void generateWaypointMarkers(const waypoint_msgs::WaypointList& wps, visualization_msgs::MarkerArray& wp_markers);
      void generateTrajectoryMarkers(const waypoint_msgs::TrajectoryList& trajs,
                                     visualization_msgs::MarkerArray& traj_markers);
      void createMarkerArrow(const int i, const waypoint_msgs::Waypoint& wp, visualization_msgs::Marker& marker);
      void createMarkerLineStrip(const int i, const waypoint_msgs::Trajectory& traj, visualization_msgs::Marker& marker);
      void createMarkerLabel(const std::string frame_id,
                             const int id,
                             const std::string ns,
                             const std::string wp_name,
                             const geometry_msgs::Pose wp_pose,
                             visualization_msgs::Marker& marker);
      void getYamlNode(const std::string& filename, YAML::Node& node);
      void parseWaypoints(const YAML::Node& node, waypoint_msgs::WaypointList& wps);
      void parseTrajectories(const YAML::Node& node,
                       const waypoint_msgs::WaypointList& wps,
                       waypoint_msgs::TrajectoryList& trajs);

    private:
      bool initialized_;
      ros::NodeHandle* nh_;
      ros::Publisher waypoints_pub_, trajectories_pub_;
      ros::Publisher waypoints_marker_pub_, trajectory_marker_pub_;
      ros::ServiceServer waypoints_srv_;
      ros::ServiceServer trajectories_srv_;

      waypoint_msgs::WaypointList waypoints_;
      waypoint_msgs::TrajectoryList trajectories_;
      visualization_msgs::MarkerArray waypoint_markers_, trajectory_markers_;

      std::string currentTrajectory;
      int currentTrajectoryID=-1;
      std::string currentWaypoint;
      int currentWaypointID = -1;

      int marker_index_;
      int label_index_;
  };


#endif // WAYPOINT_PROVIDER_H
