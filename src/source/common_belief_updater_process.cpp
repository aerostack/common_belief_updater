#include "common_belief_updater_process.h"

void CommonBeliefUpdaterProcess::ownSetUp() {
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("pose_topic", pose_topic, "self_localization/pose");
  private_nh.param<std::string>("battery_topic", battery_topic, "sensor_measurement/battery_state");
  //private_nh.param<std::string>("message_from_robot", message_from_robot,"message_from_robot");
  private_nh.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");
  //private_nh.param<std::string>("shared_robot_positions_channel_topic", shared_robot_positions_channel_str,
                                 //"shared_robot_positions_channel");
}

void CommonBeliefUpdaterProcess::ownStart() {
  pose_subscriber = n.subscribe(pose_topic, 1, &CommonBeliefUpdaterProcess::poseCallback, this);
  battery_subscriber = n.subscribe(battery_topic, 1, &CommonBeliefUpdaterProcess::batteryCallback, this);
  /*message_from_robot_sub =n.subscribe('/' + message_from_robot, 100,
                            &CommonBeliefUpdaterProcess::message_from_robotCallback, this);
  shared_robot_positions_channel_sub =
      n.subscribe('/' + shared_robot_positions_channel_str, 1000, &CommonBeliefUpdaterProcess::sharedRobotPositionCallback, this);*/
  add_client = n.serviceClient<belief_manager_msgs::AddBelief>("add_belief");
  remove_client = n.serviceClient<belief_manager_msgs::RemoveBelief>("remove_belief");
  query_client = n.serviceClient<belief_manager_msgs::QueryBelief>("query_belief");
  generate_id_client = n.serviceClient<belief_manager_msgs::GenerateID>("belief_manager_process/generate_id");

//It's needed to wait at least 2 seconds because if you don't wait it is possible that the clients are not connected.
  ros::Duration(2).sleep();
//Getting the new id for the drone
  belief_manager_msgs::GenerateID::Request req;
  belief_manager_msgs::GenerateID::Response res;
  generate_id_client.call(req, res);
  if (res.ack)
  {
    my_id = res.id;
  }
  belief_manager_msgs::QueryBelief srv;
  srv.request.query = "object(?x,drone), name(?x,"+drone_id_namespace+"), self(?x)"; 
  query_client.call(srv);
  belief_manager_msgs::QueryBelief::Response response= srv.response;
  if(response.success==false){
  	belief_manager_msgs::AddBelief srv2;
  	std::stringstream s;
  	s << "object(" << my_id << ", drone), name(" << my_id << ","+drone_id_namespace+"), self(" << my_id << ")";//llamar a generate id
 	srv2.request.belief_expression = s.str();
 	srv2.request.multivalued = false;
 	add_client.call(srv2);
 }
}

void CommonBeliefUpdaterProcess::ownStop() {
  pose_subscriber.shutdown();
  battery_subscriber.shutdown();
}

void CommonBeliefUpdaterProcess::ownRun() {}

void CommonBeliefUpdaterProcess::poseCallback(const geometry_msgs::PoseStamped& pos) {
  std::string new_flight_state;
  if(pos.pose.position.z < 0.1) {
    new_flight_state = "LANDED";
  } else {
    new_flight_state = "FLYING";
  }
  if(current_flight_state != new_flight_state) {
    bool success = sendFlightState(new_flight_state);

    if(success) {
      current_flight_state = new_flight_state;
    }
  }
  Point new_pose(pos.pose.position.x, pos.pose.position.y, pos.pose.position.z);
  new_pose.roundTo(POSE_MIN_DISTANCE);
  if(current_pose.maxDifference(new_pose) > 0) {
    bool success = sendPose(new_pose);
    if(success) {
      current_pose = new_pose;
    }
    //check if its not the first time
    if( !(last_positions.find(my_id) == last_positions.end())){
      //calculate velocity
      int32_t d_time=abs(int(pos.header.stamp.sec-last_positions[my_id].second));
      if(d_time>0.1){
      double x_now = pos.pose.position.x;
      double x_previous = last_positions[my_id].first.x;
      double d_x= x_now-x_previous; 
      double vel_x = d_x/(double)d_time;
      double y_now = pos.pose.position.y;
      double y_previous = last_positions[my_id].first.y;
      double d_y= y_now-y_previous; 
      double vel_y = d_y/(double)d_time;
      double z_now = pos.pose.position.z;
      double z_previous = last_positions[my_id].first.z;
      double d_z= z_now-z_previous; 
      double vel_z = d_z/(double)d_time;
      vel.x=vel_x;
      vel.y=vel_y;
      vel.z=vel_z;
      //update last position of the own drone
      geometry_msgs::Point p;
      p.x=pos.pose.position.x;
      p.y=pos.pose.position.y;
      p.z=pos.pose.position.y;
      std::pair <geometry_msgs::Point, int32_t> pair (p, pos.header.stamp.sec);
      last_positions[my_id]=pair;
    }
    }else{  
    //update last position of the own drone
    geometry_msgs::Point p;
    p.x=pos.pose.position.x;
    p.y=pos.pose.position.y;
    p.z=pos.pose.position.y;
    std::pair <geometry_msgs::Point, int32_t> pair (p, pos.header.stamp.sec);
    last_positions[my_id]=pair;
    }
  }
}

void CommonBeliefUpdaterProcess::batteryCallback(const  sensor_msgs::BatteryState& battery) {
    std::cout<<"new battery"<<std::endl;
  std::string new_battery_level;
  if(battery.percentage*100 < BATTERY_LOW_THRESHOLD) {
    new_battery_level = "LOW";
  } else if(battery.percentage*100 < BATTERY_MEDIUM_THRESHOLD) {
    new_battery_level = "MEDIUM";
  } else {
    new_battery_level = "HIGH";
  }

  if(new_battery_level != current_battery_level) {
    bool success = sendBatteryLevel(new_battery_level);

    if(success) {
      current_battery_level = new_battery_level;
    }
  }
}

bool CommonBeliefUpdaterProcess::sendFlightState(std::string flight_state) {
  belief_manager_msgs::AddBelief::Request req;
  belief_manager_msgs::AddBelief::Response res;

  std::stringstream ss;
  ss << "flight_state(" << my_id << ", " << flight_state << ")";
  req.belief_expression = ss.str();
  req.multivalued = false;

  add_client.call(req, res);

  return res.success;
}

bool CommonBeliefUpdaterProcess::sendPose(Point pose) {
  belief_manager_msgs::AddBelief::Request req;
  belief_manager_msgs::AddBelief::Response res;
  if(pose.x==-0){
    pose.x=0;
  }
  if(pose.y==-0){
    pose.y=0;
  }
  if(pose.z==-0){
    pose.z=0;
  }
  std::stringstream ss;
  ss << "position(" << my_id << ", (" << pose.x << ", " << pose.y << ", " << pose.z << "))";
  req.belief_expression = ss.str();
  req.multivalued = false;
  add_client.call(req, res);
  return res.success;
}

bool CommonBeliefUpdaterProcess::sendBatteryLevel(std::string level) {
  belief_manager_msgs::AddBelief::Request req;
  belief_manager_msgs::AddBelief::Response res;
  std::stringstream ss;
  ss << "battery_level(" << my_id << ", " << level << ")";
  req.belief_expression = ss.str();
  req.multivalued = false;
  add_client.call(req, res);
  return res.success;
}

CommonBeliefUpdaterProcess::Point::Point(double x_coord, double y_coord, double z_coord) {
  x = x_coord;
  y = y_coord;
  z = z_coord;
}

CommonBeliefUpdaterProcess::Point::Point() {
  x = 0;
  y = 0;
  z = 0;
}

double CommonBeliefUpdaterProcess::Point::maxDifference(CommonBeliefUpdaterProcess::Point p) {
  double x_diff = (x - p.x) > 0? x - p.x: p.x - x;
  double y_diff = (y - p.y) > 0? y - p.y: p.y - y;
  double z_diff = (z - p.z) > 0? z - p.z: p.z - z;
  double max = 0;
  for(double d: {x_diff, y_diff, z_diff}) {
    if(d > max) {
      max = d;
    }
  }
  return max;
}

void CommonBeliefUpdaterProcess::Point::roundTo(double value) {
  x = std::round(x/value)*value;
  y = std::round(y/value)*value;
  z = std::round(z/value)*value;
}
