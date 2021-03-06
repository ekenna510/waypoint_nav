    #include "waypointnav.h"

  waypointnav::waypointnav(ros::NodeHandle* nodehandle):nh(*nodehandle),move_base_ac("move_base",true)
  {
    std::string filename;
    // ROS_INFO("z");
    if(!nh.param("/waypointnav/filename", filename,std::string("/home/robot/catkin_ws/src/waypoint_nav/config/example.yaml"))) {
      ROS_ERROR("Waypoint Provider : filename argument is not set");
    }
    ROS_INFO("filename %s",filename.c_str());
    if (!waypointnav::loadWaypointsAndTrajectoriesFromYaml(filename))
    {
      ROS_ERROR("Waypointnav: failed");
    }

    nh.param("/waypointnav/frequency",      frequency,     1.0);
    nh.param("/waypointnav/close_enough",   close_enough,  0.3);  // close enough to next waypoint
    nh.param("/waypointnav/goal_timeout",   goal_timeout, 30.0);  // maximum time to reach a waypoint
    nh.param("/waypointnav/robot_frame",    robot_frame,    std::string("base_footprint"));
    nh.param("/waypointnav/world_frame",    world_frame,    std::string("map"));

    ROS_INFO("goal_time %lf",goal_timeout);

  //  waypoints_pub = nh.advertise<waypoint_msgs::WaypointList>("waypoints", 5, true);
  //  trajectories_pub = nh.advertise<waypoint_msgs::TrajectoryList>("trajectories", 5, true);
    waypoints_marker_pub = nh.advertise<visualization_msgs::Marker>("waypoint_markers", 5, true);
  //  trajectory_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 5, true);
    // setup srv server
  //  waypoints_srv = nh.advertiseService("request_waypoints", &waypointnav::processWaypointsService, this);
    // setup sound message
    sound_pub = nh.advertise<turtlebot3_msgs::Sound>("sound",5,true);
    sound_.value = 3;

    waypoints_ = wps;
    trajectories_ = trajs;
  //  while ((move_base_ac.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
  //  {
  //    ROS_WARN_THROTTLE(1, "Waiting for move_base action server to come up...");
  //  }

   state_ = IDLE;
   mode_ = GOAL;
   // generateWaypointMarkers(waypoints_, waypoint_markers_);
   // generateTrajectoryMarkers(trajectories_, trajectory_markers_);

    marker_index_ = 1000;
    label_index_ = 2000;
    tf_listener_= new tf2_ros::TransformListener(tfBuffer);
  
  }

  bool waypointnav::loadWaypointsAndTrajectoriesFromYaml(const std::string& filename)
  {
   // ROS_INFO("a");

    wps.waypoints.clear();
    trajs.trajectories.clear();

    // Yaml File Parsing
    try
    {
      YAML::Node doc;

      waypointnav::getYamlNode(filename, doc);


      waypointnav::parseWaypoints(doc, wps);

      waypointnav::parseTrajectories(doc, wps, trajs);
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

  void waypointnav::getYamlNode(const std::string& filename, YAML::Node& node)
  {
    std::ifstream ifs(filename.c_str(), std::ifstream::in);
    if (ifs.good() == false)
    {
      throw std::string("Waypoints file not found");
    }


    node = YAML::Load(ifs);
  }

  void waypointnav::parseWaypoints(const YAML::Node& node, waypoint_msgs::WaypointList& wps)
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


        wps.waypoints.push_back(wp);
      }
      ROS_INFO_STREAM("Parsed " << wps.waypoints.size() << " waypoints.");
    }
    else
    {
      ROS_WARN_STREAM("Couldn't find any waypoints in the provided yaml file.");
    }

  }

  void waypointnav::parseTrajectories(const YAML::Node& node,
                         const waypoint_msgs::WaypointList& wps,
                         waypoint_msgs::TrajectoryList& trajs)
  {
    unsigned int i;

    const YAML::Node& wp_node_tmp = node["trajectories"];
    const YAML::Node* wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
    if(wp_node != NULL)
    {
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
          for(unsigned int known_wp = 0; known_wp < wps.waypoints.size(); ++known_wp)
          {
            if (wp_name == wps.waypoints[known_wp].name)
            {
              traj.waypoints.push_back(wps.waypoints[known_wp]);
              wp_found = true;
              break;
            }
          }
          if (!wp_found)
          {
            all_waypoints_found = false;
            break;
          }
        }
        if (all_waypoints_found)
        {
          traj.name= (*wp_node)[i]["name"].as<std::string>(); // >> traj.name;
          trajs.trajectories.push_back(traj);
        }
      }
      ROS_INFO_STREAM("Parsed " << trajs.trajectories.size() << " trajectories.");
    }
    else
    {
      ROS_WARN_STREAM("Couldn't find any trajectories in the provided yaml file.");
    }

  }

  // The >> operator disappeared in yaml-cpp 0.5, so this function is
  // added to provide support for code written under the yaml-cpp 0.3 API.
  template<typename T>
  void operator >> (const YAML::Node& node, T& i)
  {
    i = node.as<T>();
  }


  /*
  bool waypointnav::processWaypointsService(waypoint_msgs::WaypointListService::Request& request,
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
  */
  void waypointnav::generateWaypointMarkers(const waypoint_msgs::WaypointList& wps,
                                                 visualization_msgs::MarkerArray& wp_viz)
  {
    wp_viz.markers.clear();

    unsigned int i;

    for(i = 0; i < wps.waypoints.size(); i++)
    {
      visualization_msgs::Marker marker;
      visualization_msgs::Marker label;

      createMarkerArrow(i, wps.waypoints[i], marker);
      createMarkerLabel(wps.waypoints[i].header.frame_id,
                        i,
                        "waypoint_labels",
                        wps.waypoints[i].name,
                        wps.waypoints[i].pose,
                        label);

      wp_viz.markers.push_back(marker);
      wp_viz.markers.push_back(label);
    }
  }

  void waypointnav::generateTrajectoryMarkers(const waypoint_msgs::TrajectoryList& trajs,
                                                   visualization_msgs::MarkerArray& traj_markers)
  {
    traj_markers.markers.clear();
    for(unsigned int traj = 0; traj < trajs.trajectories.size(); ++traj)
    {
      visualization_msgs::Marker marker;
      createMarkerLineStrip(traj, trajs.trajectories[traj], marker);
      traj_markers.markers.push_back(marker);
      // no label, since way points with labels are already generated by for the waypoint marker publisher
    }
  }

  void waypointnav::createMarkerArrow(const int i,
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
    marker.lifetime = ros::Duration();

  }

  void waypointnav::createMarkerLabel(const std::string frame_id,
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
    marker.lifetime = ros::Duration();

  }

  void waypointnav::createMarkerLineStrip(const int i,
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
  /*
  void waypointnav::cspin() {
   // waypoints_pub.publish(waypoints_);
   // trajectories_pub.publish(trajectories_);
   // waypoints_marker_pub.publish(waypoint_markers_);
   // trajectory_marker_pub.publish(trajectory_markers_);
    initialized_ = true;
    ros::spin();



  }
  */
  void waypointnav::resetWaypoints()
  {
    ROS_INFO("Full reset: clear markers, delete waypoints and goal and set state to IDLE");
    //waypoints_.clear();
    //waypoints_it_ = waypoints_.end();
    //goal_  = NOWHERE;
    mode_  = NONE;
  }

//  double waypointnav::distance2D(geometry_msgs::Pose a, geometry_msgs::Pose b)
//  {
//    return distance2D(a.position, b.position);
//  }


//  double waypointnav::distance2D(const tf::Transform& a, const tf::Transform& b)
//  {
//    return waypointnav::distance2D(a.getOrigin(), b.getOrigin());
//  }


//  double waypointnav::distance2D(double x, double y)
//  {
//    return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
//  }

//  double waypointnav::distance2D(const tf::Point& p)
//  {
//    return std::sqrt(std::pow(p.x(), 2) + std::pow(p.y(), 2));
//  }


  double waypointnav::distance2D(double ax, double ay, double bx, double by)
  {
    return std::sqrt(std::pow(ax - bx, 2) + std::pow(ay - by, 2));
  }

//  double waypointnav::distance2D(const tf::Point& p1, const tf::Point& p2)
//  {
//    return std::sqrt(std::pow(p2.x() - p1.x(), 2) + std::pow(p2.y() - p1.y(), 2));
//  }

//  double waypointnav::distance2D(geometry_msgs::Point a, geometry_msgs::Point b)
//  {
//    return distance2D(tf::Vector3(a.x, a.y, a.z), tf::Vector3(b.x, b.y, b.z));
//  }

  double waypointnav::calcDistanceToGoal(move_base_msgs::MoveBaseGoal mb_goal)
  {
    geometry_msgs::TransformStamped robot_gb;


    try
      {
      robot_gb = tfBuffer.lookupTransform(world_frame, robot_frame,ros::Time(0.0));
      }
    catch (tf2::TransformException& e)
      {
        ROS_WARN("Cannot get tf %s -> %s: %s", world_frame.c_str(), robot_frame.c_str(), e.what());
      }
    
    double distance = distance2D(robot_gb.transform.translation.x,robot_gb.transform.translation.y,mb_goal.target_pose.pose.position.x,mb_goal.target_pose.pose.position.y);
    return distance;

  }

  void waypointnav::spin()
  {
    int i=0;
    move_base_msgs::MoveBaseGoal mb_goal;

    ros::Rate rate(frequency);
    state_ = START;
    waypoint_msgs::Trajectory traj = trajs.trajectories[0];
    while (ros::ok())
    {
      rate.sleep();
      ros::spinOnce();

      if (state_ == START)
      {
  //      if (mode == LOOP)
  //      {
  //        if (waypoints_it == waypoints.end())
  //        {
  //          waypoints_it = waypoints.begin();
  //        }
  //      }
 //-------------------------------

        visualization_msgs::Marker marker;
        visualization_msgs::Marker label;

        createMarkerArrow(i, traj.waypoints[i], marker);
        createMarkerLabel(traj.waypoints[i].header.frame_id,
                          i,
                          "waypoint_labels",
                          traj.waypoints[i].name,
                          traj.waypoints[i].pose,
                          label);
          waypoints_marker_pub.publish(marker);
          waypoints_marker_pub.publish(label);
//------------------------------



        if (i < traj.waypoints.size() )
        {
          mb_goal.target_pose.header.stamp = ros::Time::now();
          mb_goal.target_pose.header.frame_id = traj.waypoints[i].header.frame_id;
          mb_goal.target_pose.pose = traj.waypoints[i].pose;
  //        mb_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);  // TODO use the heading from robot loc to next (front)

          ROS_INFO("New goal: %.2f, %.2f, ",
                   mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y
                   //tf::getYaw(mb_goal.target_pose.pose.orientation)
                   );
          move_base_ac.sendGoal(mb_goal);


          distanceToGoal = calcDistanceToGoal(mb_goal);

  //        publishStatusUpdate(yocs_msgs::NavigationControlStatus::RUNNING);


          state_ = ACTIVE;
        }
        else
        {
          ROS_ERROR_STREAM("Cannot start execution. Already at the last way point.");
      //    idle_status_update_sent_ = false;
          state_ = IDLE;
        }

        // TODO: This is a horrible workaround for a problem I cannot solve: send a new goal
        // when the previous one has been cancelled return immediately with succeeded state
        //
        // Marcus: Don't understand this case (yet). Commenting out until we need it.
  //        int times_sent = 0;
  //        while ((move_base_ac_.waitForResult(ros::Duration(0.1)) == true) &&
  //               (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
  //        {
  //          move_base_ac_.sendGoal(mb_goal);
  //          times_sent++;
  //        }
  //        if (times_sent > 1)
  //        {
  //          ROS_WARN("Again the strange case of instantaneous goals... (goal sent %d times)", times_sent);
  //        }
      }
      else if (state_ == ACTIVE)
      {
        actionlib::SimpleClientGoalState goal_state = move_base_ac.getState();

       // actionlib::SimpleClientGoalState goal_state = move_base_ac.getResult

       //move_base_msgs::MoveBaseResultConstPtr actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> const myresult;
       //myresult = move_base_ac.getResult();
        // We are still pursuing a goal...
        if ((goal_state == actionlib::SimpleClientGoalState::ACTIVE) ||
            (goal_state == actionlib::SimpleClientGoalState::PENDING) ||
            (goal_state == actionlib::SimpleClientGoalState::RECALLED) ||
            (goal_state == actionlib::SimpleClientGoalState::PREEMPTED))
        {
          // check if we timed out
          if ((ros::Time::now() - mb_goal.target_pose.header.stamp).toSec() >= goal_timeout)
          {

            ROS_WARN("Cannot reach goal after %.2f seconds; request a new one (current state is %s)",
                      goal_timeout, move_base_ac.getState().toString().c_str());

            ROS_INFO("timeout at : %.2f, %.2f,",
                     mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y
                     //tf::getYaw(mb_goal.target_pose.pose.orientation)
                     );

            double currentDistance =calcDistanceToGoal(mb_goal);
            
            // have we moved at least .1
            if (distanceToGoal> currentDistance && distanceToGoal- currentDistance > .1 )
            {
            // continue
            // reset timer and distance to goal
            mb_goal.target_pose.header.stamp= ros::Time::now();
            distanceToGoal=currentDistance;
            }
            else
            {
                //stuck
              if (i < (traj.waypoints.size() - 1))
              {
                ROS_INFO("stuck distancetogoal %lf currentdistance %lf",distanceToGoal,currentDistance);
                //i++;
                //state_ = START;
              }
              else
              {
                ROS_INFO("No more way points to go to. end at: %.2f, %.2f,",
                        mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y
                        //tf::getYaw(mb_goal.target_pose.pose.orientation)
                        );
                state_ = COMPLETED;
              }

            }
            
          }
          // When close enough to current goal (except for the final one!), go for the
          // next waypoint, so we avoid the final slow approach and subgoal obsession
          if (i < (traj.waypoints.size()-1) )
          {

              // keep going until get close enough

          }
          else
          {
            // keep going, since we approaching last way point
          }
        }
        else // actionlib::SimpleClientGoalState::SUCCEEDED, REJECTED, ABORTED, LOST
        {
          if (goal_state == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("Go to goal successfully completed: %.2f, %.2f ",
                     mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y
                     //tf::getYaw(mb_goal.target_pose.pose.orientation)
                     );
            if (i < (traj.waypoints.size() - 1))
            {
              ROS_INFO_STREAM("Requesting next way point.");
              i++;
              state_ = START;
            }
            else
            {
              ROS_INFO_STREAM("Reached final way point.");
              state_ = COMPLETED;
            }
            sound_pub.publish(sound_);
          }
          else
          {
            ROS_ERROR("Go to goal failed: %s at  %.2f, %.2f  because %s", move_base_ac.getState().toString().c_str(),mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
//                     tf::getYaw(mb_goal.target_pose.pose.orientation),
                     goal_state.getText().c_str() );
            if (i < (traj.waypoints.size() - 1))
            {
              ROS_INFO_STREAM("Requesting next way point.");
              i++;
              state_ = START;
            }
            else
            {
              ROS_INFO_STREAM("No more way points to go to.");
              state_ = COMPLETED;
            }
          }
        }
      }
      else if(state_ == COMPLETED)
      {
        // publish update
        //publishStatusUpdate(yocs_msgs::NavigationControlStatus::COMPLETED);
        idle_status_update_sent = false;
        state_ = IDLE;
      }
      else // IDLE
      {
        if (!idle_status_update_sent)
        {
         // publishStatusUpdate(yocs_msgs::NavigationControlStatus::IDLING);
          idle_status_update_sent = true;
        }
      }
    }
  }





  int main(int argc, char** argv)
  {

    ros::init(argc, argv, "waypointnav");



    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    //("Y");

    ROS_DEBUG("main: instantiating an object of type waypointnav");
    waypointnav waypointnav(&nh);  //instantiate an waypointnav object and pass in pointer to nodehandle for constructor to use
   // ROS_INFO("x");
    waypointnav.spin();
    return 0;



  }
//  void waypointnav::pose2tf(const geometry_msgs::Pose& pose, tf::Transform& tf)
//  {
//    tf.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
//    tf::Quaternion q;
//    tf:quaternionMsgToTF(pose.orientation, q);
//    tf.setRotation(q);
//  }

//  void waypointnav::pose2tf(const geometry_msgs::PoseStamped& pose, tf::StampedTransform& tf)
//  {
//    tf.stamp_    = pose.header.stamp;
//    tf.frame_id_ = pose.header.frame_id;
//   pose2tf(pose.pose, tf);
//  }

