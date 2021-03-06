/*
   Way point Provider

   inspired by yocs_waypoints_navi

   LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE

   Author : Jihoon Lee
   Date   : Dec 2013
 */

#include "waypoint_provider.h"
#include "yaml_parser.h"
#include "WaypointList.h"



  WaypointProvider::WaypointProvider(ros::NodeHandle* n)

  {

//: nh_(n)
    nh_=n;
    // setup pub
    waypoints_pub_ = nh_->advertise<waypoint_msgs::WaypointList>("waypoints", 5, true);
    trajectories_pub_ = nh_->advertise<waypoint_msgs::TrajectoryList>("trajectories", 5, true);
    waypoints_marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("waypoint_markers", 5, true);
    trajectory_marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("trajectory_markers", 5, true);
    // setup srv server
    waypoints_srv_ = nh_->advertiseService("request_waypoints", &WaypointProvider::processWaypointsService, this);
    trajectories_srv_ = nh_->advertiseService("request_trajectory", &WaypointProvider::processTrajectoryService, this);

    //waypoints_ = waypoints_;
    //trajectories_ = trajectories_;
    //generateWaypointMarkers(waypoints_, waypoint_markers_);
    //generateTrajectoryMarkers(trajectories_, trajectory_markers_);

    marker_index_ = 1000;
    label_index_ = 2000;
  }


  WaypointProvider::~WaypointProvider() {}

  bool WaypointProvider::processWaypointsService(waypoint_msgs::WaypointListService::Request& request,
                                                 waypoint_msgs::WaypointListService::Response& response)
  {
    ROS_INFO("Waypoint Manager : Received request");
    if(!initialized_) // return false if node is not initialized with points
    {
      response.success = false;
    }
    else {
      response.waypoints = this->waypoints_;
      response.success = true;
    }
    return true;
  }

  bool WaypointProvider::processTrajectoryService(waypoint_msgs::TrajectoryService::Request& request,
  waypoint_msgs::TrajectoryService::Response& response  )
  {
    ROS_INFO("Trajectory Manager : Received request");
    if(initialized_) // return false if node is not initialized with points
    {
    if ( currentTrajectoryID < 0 || request.trajectoryname != currentTrajectory)
      {
        currentTrajectoryID = -1;
        ROS_INFO("size %ld", trajectories_.trajectories.size());
        for (int i=0; i <= trajectories_.trajectories.size(); i++ )
        {
          ROS_DEBUG("name %d %s",i,trajectories_.trajectories[i].name.c_str());
            if (trajectories_.trajectories[i].name == request.trajectoryname)
              {
               ROS_INFO("Trajectory Manager : New Trajectory %s found",trajectories_.trajectories[i].name.c_str());
              currentTrajectory = request.trajectoryname;
              currentTrajectoryID = i;
              currentWaypointID = -1;
              break;
              }
        }
      }
    else
      {
      ROS_INFO("Trajectory Manager : using Trajectory %s",trajectories_.trajectories[currentTrajectoryID].name.c_str());
      }
    
    if ( currentTrajectoryID > -1 )
    {
      ROS_DEBUG("lastwaypoint requested %s",request.lastwaypoint.c_str());
      // if no waypoint name supplied advance to next waypoint
      if (request.lastwaypoint.empty())
        {
        currentWaypointID++;
        ROS_DEBUG("Empty waypoint getting next waypoint %d",currentWaypointID);

        response.success = true;
        }
      else
        {
        
        if (currentWaypointID > -1 && trajectories_.trajectories[currentTrajectoryID].waypoints[currentWaypointID].name == request.lastwaypoint  )
          {
            ROS_DEBUG("current waypointid matched %s going to next waypoint",request.lastwaypoint.c_str());
            currentWaypointID++;
            response.success = true;
          }
        else
          {
          int i;
          // search for the last waypoint
          for (i=0;i<trajectories_.trajectories[currentTrajectoryID].waypoints.size();i++)
            {
            if (trajectories_.trajectories[currentTrajectoryID].waypoints[i].name == request.lastwaypoint )
              {
              ROS_DEBUG("found  %s going to next waypoint",request.lastwaypoint.c_str());
              currentWaypointID= i+1;
              response.success = true;
              break;
              }
            }
          if ( i ==trajectories_.trajectories[currentTrajectoryID].waypoints.size()  )
            {
            ROS_WARN("Could not find the last waypoint %s",request.lastwaypoint.c_str());
            response.success = false;
            }
          }
        }

        if (currentWaypointID < trajectories_.trajectories[currentTrajectoryID].waypoints.size() && response.success )
          {
          // default to false
          response.endoftrajectory = false;
          ROS_DEBUG("currentWaypointID %d",currentWaypointID);
          response.waypoint = trajectories_.trajectories[currentTrajectoryID].waypoints[currentWaypointID];
          if (currentWaypointID ==trajectories_.trajectories[currentTrajectoryID].waypoints.size()-1)
            {
              // we are at the end
              ROS_INFO("At the last waypoint %d, ",currentWaypointID);
              response.endoftrajectory = true;
            }

          }
        else
        {
          ROS_WARN("Trying to go beyond the last waypoint");
          response.success = false;

        }
        

    }
    else
    {
      ROS_WARN("Trajectory Manager : Trajestory %s not found",request.trajectoryname.c_str());
      response.success = false;
    }


    }
    else {
      ROS_WARN("Trajectory Manager : Not initialized");
      response.success = false;
    }
    return true;

  }
  void WaypointProvider::generateWaypointMarkers(const waypoint_msgs::WaypointList& waypoints_,
                                                 visualization_msgs::MarkerArray& wp_viz)
  {
    wp_viz.markers.clear();

    unsigned int i;

    for(i = 0; i < waypoints_.waypoints.size(); i++)
    {
      visualization_msgs::Marker marker;
      visualization_msgs::Marker label;

      createMarkerArrow(i, waypoints_.waypoints[i], marker);
      createMarkerLabel(waypoints_.waypoints[i].header.frame_id,
                        i,
                        "waypoint_labels",
                        waypoints_.waypoints[i].name,
                        waypoints_.waypoints[i].pose,
                        label);

      wp_viz.markers.push_back(marker);
      wp_viz.markers.push_back(label);
    }
  }

  void WaypointProvider::generateTrajectoryMarkers(const waypoint_msgs::TrajectoryList& trajectories_,
                                                   visualization_msgs::MarkerArray& traj_markers)
  {
    traj_markers.markers.clear();
    for(unsigned int traj = 0; traj < trajectories_.trajectories.size(); ++traj)
    {
      visualization_msgs::Marker marker;
      createMarkerLineStrip(traj, trajectories_.trajectories[traj], marker);
      traj_markers.markers.push_back(marker);
      // no label, since way points with labels are already generated by for the waypoint marker publisher
    }
  }

  void WaypointProvider::createMarkerArrow(const int i,
                                           const waypoint_msgs::Waypoint& wp,
                                           visualization_msgs::Marker& marker)
  {
    marker.header.frame_id = wp.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "waypoints";
    marker.id = i + marker_index_;
    marker.pose = wp.pose;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

  }

  void WaypointProvider::createMarkerLabel(const std::string frame_id,
                                           const int id,
                                           const std::string ns,
                                           const std::string wp_name,
                                           const geometry_msgs::Pose wp_pose,
                                           visualization_msgs::Marker& marker)
  {
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = id + label_index_;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    marker.pose = wp_pose;
    marker.pose.position.z = marker.pose.position.z + marker.scale.z / 2.0 + 0.05;  // just above the marker
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.text = wp_name;
  }

  void WaypointProvider::createMarkerLineStrip(const int i,
                                               const waypoint_msgs::Trajectory& traj,
                                               visualization_msgs::Marker& marker)
  {
    marker.header.frame_id = traj.waypoints[0].header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "trajectories";
    marker.id = i + marker_index_;
    marker.pose = geometry_msgs::Pose();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    for(unsigned int wp = 0; wp < traj.waypoints.size(); ++wp)
    {
      marker.points.push_back(traj.waypoints[wp].pose.position);
      marker.colors.push_back(marker.color);
    }
  }

  void WaypointProvider::spin() {
    waypoints_pub_.publish(waypoints_);
    trajectories_pub_.publish(trajectories_);
    waypoints_marker_pub_.publish(waypoint_markers_);
    trajectory_marker_pub_.publish(trajectory_markers_);
    initialized_ = true;
    ros::spin();
  }

bool WaypointProvider::loadWaypointsAndTrajectoriesFromYaml(const std::string& filename)
{

  waypoints_.waypoints.clear();
  trajectories_.trajectories.clear();
  currentTrajectoryID=-1;
  currentWaypointID=-1;


  // Yaml File Parsing
  try
  {
    YAML::Node doc;

    WaypointProvider::getYamlNode(filename, doc);


    WaypointProvider::parseWaypoints(doc, waypoints_);

    WaypointProvider::parseTrajectories(doc, waypoints_, trajectories_);
  }
  catch(YAML::ParserException& e)
  {
    ROS_ERROR("Parsing waypoints file failed: %s", e.what());
    return false;
  }
  catch(YAML::RepresentationException& e)
  {
    ROS_ERROR("Parsing waypoints file failed: %s", e.what());
    return false;
  }
  catch(std::string& e) {
    ROS_ERROR("Parsing waypoints file failed: %s",e.c_str());
    return false;
  }

  return true;
}
void WaypointProvider::getYamlNode(const std::string& filename, YAML::Node& node)
{
  std::ifstream ifs(filename.c_str(), std::ifstream::in);
  if (ifs.good() == false)
  {
    throw std::string("Waypoints file not found");
  }


  node = YAML::Load(ifs);
}


void WaypointProvider::parseWaypoints(const YAML::Node& node, waypoint_msgs::WaypointList& waypoints_)
{

    const YAML::Node& wp_node_tmp = node["waypoints"];
    const YAML::Node* wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
    std::string tempfloat;
  if(wp_node != NULL)
  {
    for(unsigned int i = 0; i < wp_node->size(); ++i)
    {
      // Parse waypoint entries on YAML
      waypoint_msgs::Waypoint wp;

      wp.name = (*wp_node)[i]["name"].as<std::string>();// >> wp.name;

      wp.header.frame_id = (*wp_node)[i]["frame_id"].as<std::string>();// >> wp.header.frame_id;
      tempfloat =(*wp_node)[i]["pose"]["position"]["x"].as<std::string>();
      wp.pose.position.x = std::stof(tempfloat);// >> wp.pose.position.x;
      tempfloat = (*wp_node)[i]["pose"]["position"]["y"].as<std::string>();// >> wp.pose.position.y;
      wp.pose.position.y = std::stof(tempfloat);
      tempfloat = (*wp_node)[i]["pose"]["position"]["z"].as<std::string>();// >> wp.pose.position.z;
      wp.pose.position.z = std::stof(tempfloat);

      tempfloat = (*wp_node)[i]["pose"]["orientation"]["x"].as<std::string>();// >> wp.pose.orientation.x;
      wp.pose.orientation.x = std::stof(tempfloat);

      tempfloat = (*wp_node)[i]["pose"]["orientation"]["y"].as<std::string>();// >> wp.pose.orientation.y;
      wp.pose.orientation.y = std::stof(tempfloat);

      tempfloat = (*wp_node)[i]["pose"]["orientation"]["z"].as<std::string>();// >> wp.pose.orientation.z;
      wp.pose.orientation.z = std::stof(tempfloat);

      tempfloat = (*wp_node)[i]["pose"]["orientation"]["w"].as<std::string>();// >> wp.pose.orientation.w;
      wp.pose.orientation.w = std::stof(tempfloat);


      waypoints_.waypoints.push_back(wp);
    }
    ROS_INFO_STREAM("Parsed " << waypoints_.waypoints.size() << " waypoints.");
  }
  else
  {
    ROS_WARN_STREAM("Couldn't find any waypoints in the provided yaml file.");
  }

}

void WaypointProvider::parseTrajectories(const YAML::Node& node,
                       const waypoint_msgs::WaypointList& waypoints_,
                       waypoint_msgs::TrajectoryList& trajectories_)
{
  unsigned int i;

  const YAML::Node& wp_node_tmp = node["trajectories"];
  const YAML::Node* wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
  if(wp_node != NULL)
  {
    ROS_DEBUG("wp-node size %ld",wp_node->size());
    for(i = 0; i < wp_node->size(); ++i)
    {
      // Parse trajectory entries on YAML
      waypoint_msgs::Trajectory traj;

      // check if all specified waypoints are configured
      bool all_waypoints_found = true;

      for(unsigned int wp = 0; wp < (*wp_node)[i]["waypoints"].size(); ++wp)
      {
        bool wp_found = false;
        std::string wp_name;
        wp_name = (*wp_node)[i]["waypoints"][wp].as<std::string>();// >> wp_name;
        for(unsigned int known_wp = 0; known_wp < waypoints_.waypoints.size(); ++known_wp)
        {
          if (wp_name == waypoints_.waypoints[known_wp].name)
          {
            traj.waypoints.push_back(waypoints_.waypoints[known_wp]);
            wp_found = true;
            break;
          }
        }
        if (!wp_found)
        {
          ROS_INFO("wp-node %s notfound ",wp_name.c_str());          
          all_waypoints_found = false;
          break;
        }
      }
      if (all_waypoints_found)
      {
        traj.name= (*wp_node)[i]["name"].as<std::string>(); // >> traj.name;
        trajectories_.trajectories.push_back(traj);
      }
      else
      {
        ROS_WARN("Trajectory %s could not be loaded missing waypoints",traj.name.c_str());
      }
      
      ROS_DEBUG("wp-node i %d",i);
    }
    ROS_INFO_STREAM("Parsed " << trajectories_.trajectories.size() << " trajectories.");
  }
  else
  {
    ROS_WARN_STREAM("Couldn't find any trajectories in the provided yaml file.");
  }

}

// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
//<template<typename T>
//void operator >> (const YAML::Node& node, T& i)
//{
//  i = node.as<T>();
//}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_provider");
  ros::NodeHandle priv_n("~");
  ros::NodeHandle n;
  WaypointProvider* wm;
  //WaypointProvider wm;
  waypoint_msgs::WaypointList waypoints_;
  waypoint_msgs::TrajectoryList trajectories_;
  std::string filename;
  yaml_parser yp;

  if(!priv_n.getParam("filename", filename)) {
    ROS_ERROR("Waypoint Provider : filename argument is not set");
    return -1;
  }

   wm = new WaypointProvider(&n);

  wm->loadWaypointsAndTrajectoriesFromYaml( filename);
  ROS_INFO("Waypoint Provider : Initialized");
  wm->spin();
  ROS_INFO("Waypoint Provider : Bye Bye");

  delete wm;

  return 0;
}

