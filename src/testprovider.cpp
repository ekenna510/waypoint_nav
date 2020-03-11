#include "ros/ros.h"
#include "waypoint_msgs/WaypointListService.h"
#include "waypoint_msgs/TrajectoryService.h"





int main(int argc, char **argv)
{
  ros::init(argc, argv, "testprovider");
  ros::NodeHandle nh("~");
  ros::ServiceClient client; 
  ros::ServiceClient tclient; 
  std::string trajectoryname ="go_to_door";
  std::string waypointname="";

 
  waypoint_msgs::TrajectoryService tserv;
  waypoint_msgs::WaypointListService serv;

  client = nh.serviceClient<waypoint_msgs::WaypointListService>("/request_waypoints");
  if (client)
    {
    ROS_INFO("client ok");
    ros::service::waitForService("request_waypoints");
    }
  else 
  {
    ROS_INFO("client failed ");
  }
  tclient = nh.serviceClient<waypoint_msgs::TrajectoryService>("/request_trajectory");
  if (tclient)
    {
    ROS_INFO("tclient ok");
    ros::service::waitForService("request_trajectory");
    }
  else 
  {
    ROS_INFO("tclient failed ");
  }

bool done=false;

while (!done)
  {
  if (client.call(serv))
    {
      ROS_INFO("w success");
    }
  else
  {
    ROS_INFO("w error ");
  }
  tserv.request.trajectoryname = trajectoryname;
  tserv.request.lastwaypoint = waypointname;

  if (tclient.call(tserv))
    {
      if(tserv.response.success)
      {
      waypointname = tserv.response.waypoint.name;
      ROS_INFO("t success %s ", waypointname.c_str());
      done = tserv.response.endoftrajectory;
      }
    }
  else
  {
    ROS_INFO("t error ");
  }

    ros::spinOnce();
  }

}
