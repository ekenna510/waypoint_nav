#include "ros/ros.h"
#include "waypoint_msgs/WaypointListService.h"




int main(int argc, char **argv)
{
  ros::init(argc, argv, "testprovider");
  ros::NodeHandle nh("~");
  ros::ServiceClient client; 

    ROS_INFO("a");
  
  waypoint_msgs::WaypointListService serv;
    ROS_INFO("b ");

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

  while (ros::ok())
  {
  ROS_INFO("in while ");

  if (client.call(serv))
    {
      ROS_INFO(" success");
    }
  else
  {
    ROS_INFO("error ");
  }


    ros::spinOnce();
  }

}
