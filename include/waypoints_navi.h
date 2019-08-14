/**
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#ifndef WAYPOINT_NAVI_H
#define WAYPOINT_NAVI_H

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
/*
#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>
*/
#include <waypoint_msgs/NavigationControl.h>
#include <waypoint_msgs/NavigationControlStatus.h>
#include <waypoint_msgs/TrajectoryList.h>
#include <waypoint_msgs/WaypointList.h>




/*
 * TODO
 *  * think about how to best visualise the waypoint(s)/trajectory(ies) which are being executed
 *  * add RViz interface to yocs_waypoint_provider
 */

class WaypointsGoalNode
{
public:
  WaypointsGoalNode();
  ~WaypointsGoalNode();

  bool init();

  void waypointsCB(const waypoint_nav::WaypointList::ConstPtr& wps);

  void trajectoriesCB(const waypoint_nav::TrajectoryList::ConstPtr& trajs);

  void navCtrlCB(const waypoint_nav::NavigationControl::ConstPtr& nav_ctrl);

  void spin();
  // grabbed from mathtoolkit
  void pose2tf(const geometry_msgs::Pose& pose, tf::Transform& tf);
  void pose2tf(const geometry_msgs::PoseStamped& pose, tf::StampedTransform& tf);
  double distance2D(double ax, double ay, double bx, double by);
  double distance2D(geometry_msgs::Point a, geometry_msgs::Point b = geometry_msgs::Point());
  double distance2D(geometry_msgs::Pose a, geometry_msgs::Pose b = geometry_msgs::Pose());
  double distance2D(const tf::Vector3& a, const tf::Vector3& b = tf::Vector3());
  double distance2D(const tf::Transform& a, const tf::Transform& b = tf::Transform());

private:
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

  double      frequency_;
  double      close_enough_;
  double      goal_timeout_;
  std::string robot_frame_;
  std::string world_frame_;

  std::vector<geometry_msgs::PoseStamped>           waypoints_;
  std::vector<geometry_msgs::PoseStamped>::iterator waypoints_it_;

  geometry_msgs::PoseStamped goal_;

  waypoint_nav::WaypointList wp_list_;
  waypoint_nav::TrajectoryList traj_list_;

  tf::TransformListener tf_listener_;
  ros::Subscriber    waypoints_sub_;
  ros::Subscriber    trajectories_sub_;

  ros::Subscriber nav_ctrl_sub_;
  ros::Publisher  status_pub_;
  bool idle_status_update_sent_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;

  bool cancelAllGoals(double timeout = 2.0);

  void resetWaypoints();

  bool equals(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b);

  bool equals(const geometry_msgs::Point& a, const geometry_msgs::Point& b);


};


#endif /* WAYPOINT_NAVI_H */
