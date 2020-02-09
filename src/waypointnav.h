#ifndef WAYPOINTNAV_H
#define WAYPOINTNAV_H

#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include "TrajectoryList.h"
#include "WaypointList.h"




#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include "Trajectory.h"
#include "WaypointListService.h"
#include <fstream>
  #include <move_base_msgs/MoveBaseAction.h>
  #include <actionlib/client/simple_action_client.h>
  #include <geometry_msgs/PointStamped.h>
//#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/TransformStamped.h>
#include <turtlebot3_msgs/Sound.h>

class waypointnav
{
public:
  waypointnav(ros::NodeHandle* nodehandle);
  void spin();
protected:

  void init();
  bool processWaypointsService(waypoint_msgs::WaypointListService::Request& request,
                               waypoint_msgs::WaypointListService::Response& response);
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

//void pose2tf(const geometry_msgs::Pose& pose, tf::Transform& tf);
//void pose2tf(const geometry_msgs::PoseStamped& pose, tf::StampedTransform& tf);
//double distance2D(const tf::Transform& a, const tf::Transform& b);
//double distance2D(double x, double y);
//double distance2D(const tf::Point& p);
//double distance2D(geometry_msgs::Pose a, geometry_msgs::Pose b);

double distance2D(double ax, double ay, double bx, double by);
//double distance2D(const tf::Point& p1, const tf::Point& p2);
//double distance2D(geometry_msgs::Point a, geometry_msgs::Point b);


private:
   bool initialized_;
   bool idle_status_update_sent;
   ros::NodeHandle nh;
   waypoint_msgs::WaypointList wps;
   waypoint_msgs::TrajectoryList trajs;

   const geometry_msgs::PoseStamped NOWHERE;

   enum { NONE = 0,
          GOAL,
          LOOP
        } mode_;

   enum { IDLE = 0,
          START,
          ACTIVE,
          COMPLETED
        } state_;


   double      frequency;
   double      close_enough;
   double      goal_timeout;
   std::string robot_frame;
   std::string world_frame;
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac;

   bool cancelAllGoals(double timeout = 2.0);
   void resetWaypoints();
   tf2_ros::Buffer tfBuffer;

   tf2_ros::TransformListener *tf_listener_;

   ros::Publisher waypoints_pub, trajectories_pub;
   ros::Publisher waypoints_marker_pub, trajectory_marker_pub;
   ros::Publisher sound_pub;
   ros::ServiceServer waypoints_srv;

  bool loadWaypointsAndTrajectoriesFromYaml(const std::string& filename);
  void getYamlNode(const std::string& filename, YAML::Node& node);
  void parseWaypoints(const YAML::Node& node, waypoint_msgs::WaypointList& wps);
  void parseTrajectories(const YAML::Node& node, const waypoint_msgs::WaypointList& wps, waypoint_msgs::TrajectoryList& trajs);
  //std::vector<geometry_msgs::PoseStamped>           waypoints_;
  //std::vector<geometry_msgs::PoseStamped>::iterator waypoints_it_;
  double calcDistanceToGoal(move_base_msgs::MoveBaseGoal mb_goal);
  
  turtlebot3_msgs::Sound sound_;
  waypoint_msgs::WaypointList waypoints_;
  waypoint_msgs::TrajectoryList trajectories_;
  visualization_msgs::MarkerArray waypoint_markers_, trajectory_markers_;
  int marker_index_;
  int label_index_;
  double distanceToGoal;
};

#endif // WAYPOINTNAV_H
