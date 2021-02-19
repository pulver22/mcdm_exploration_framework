//
// Created by pulver on 20/07/19.
//

#include "include/utils.h"
#include "bayesian_topological_localisation/Predict.h"
#include "strands_navigation_msgs/GetRouteTo.h"
#include "std_msgs/String.h"

Utilities::Utilities(){
  goal_marker_.header.frame_id = "/map";
  goal_marker_.type = goal_marker_.SPHERE;
  goal_marker_.pose.position.z = 6;
  goal_marker_.pose.orientation.w = 1;
  goal_marker_.scale.x = 0.5;
  goal_marker_.scale.y = 0.5;
  goal_marker_.scale.z = 0.5;
  goal_marker_.scale.z = 1;
  goal_marker_.color.a = 1.0;
  goal_marker_.color.r = 1;
  goal_marker_.color.g = 1;
  goal_marker_.color.b = 0;
};
Utilities::~Utilities(){};

bool Utilities::contains(std::list<Pose> &list, Pose &p) {
  bool result = false;

  std::list<Pose>::iterator findIter = std::find(list.begin(), list.end(), p);
  if (findIter != list.end()) {
    // cout << "Found it: "<< function.getEncodedKey(p,0) <<endl;
    result = true;
  }
  return result;
}

bool Utilities::containsPos(
    std::list<std::pair<float, float>> *positionEscluded,
    std::pair<float, float> p) {
  bool result = false;
  std::pair<float, float> tmp_p =
      make_pair(float(int(p.first)), float(int(p.second)));
  // cout << "   [navigation_utilties.py@containsPos] tmp_p: " << tmp_p.first <<
  // "," << tmp_p.second << endl;

  auto findIter =
      std::find(positionEscluded->begin(), positionEscluded->end(), tmp_p);
  if (findIter != positionEscluded->end()) {
    result = true;
  }
  // cout << "Is it already visited and excluded: " << result << endl;
  return result;
}

void Utilities::cleanPossibleDestination2(std::list<Pose> *possibleDestinations,
                                          Pose &p) {

  //  for (auto it = possibleDestinations->begin(); it !=
  //  possibleDestinations->end(); it++)
  //  {
  //    cout << function.getEncodedKey(*it, 0) << endl;
  //  }

  std::list<Pose>::iterator findIter =
      std::find(possibleDestinations->begin(), possibleDestinations->end(), p);
  if (findIter != possibleDestinations->end()) {
    //    cout << "[navigation_utilties.cpp@cleanPossibleDestination2]
    //    EncodedKey:"
    //    <<  function.getEncodedKey(*findIter,0) << "\n" << endl;
    possibleDestinations->erase(findIter);
  }
  //  else cout<< "[navigation_utilties.cpp@cleanPossibleDestination2] Cell not
  //  found\n" << endl;

  //    for (auto it = possibleDestinations->begin(); it !=
  //    possibleDestinations->end(); it++)
  //    {
  //      cout << function.getEncodedKey(*it, 0) << endl;
  //    }

  // cout << "[navigation_utilies.cpp@cleanPossibleDestination2]
  // cleanedFronties: " << possibleDestinations->size() << endl;
}

void Utilities::cleanDestinationFromTabulist(
    std::list<Pose> *possibleDestinations,
    std::list<std::pair<float, float>> *posToEsclude) {

  std::list<Pose> finalDestination;
  // cout << "Size of possibleDestinations: " << possibleDestinations->size() <<
  // endl;
  for (auto it = possibleDestinations->begin();
       it != possibleDestinations->end(); it++) {
    // create a temporary position pair
    std::pair<float, float> tmp_position(int(it->getX()), int(it->getY()));
    // look for this position in the list of forbidden positions
    std::list<std::pair<float, float>>::iterator findIter =
        std::find(posToEsclude->begin(), posToEsclude->end(), tmp_position);
    // if it's not found, add it to the list of final Destination
    if (findIter == posToEsclude->end()) {
      finalDestination.push_back(*it);
    }
  }
  // cout << "Size of finalDestination: " << finalDestination.size() << endl;
  *possibleDestinations = finalDestination;
  // cout << "Size of possibleDestinations after update: " <<
  // possibleDestinations->size() << endl;
}

void Utilities::pushInitialPositions(string currentRobotWayPoint,
    dummy::Map map, float x, float y, float orientation, int range, int FOV,
    double threshold, string actualPose,
    vector<pair<string, list<Pose>>> *graph2, ros::ServiceClient *path_client,
    vector<unordered_map<float,
                         std::pair<string, bayesian_topological_localisation::
                                               DistributionStamped>>>
        *mapping_time_belief,
    MCDMFunction *function, double *batteryTime, GridMap *belief_map,
    unordered_map<string, string> *mappingWaypoints, prediction_tools *tools,
    std::unordered_map<string, double> *distances_map) {

  map.findCandidatePositions(x, y, orientation, FOV, range);
  vector<pair<float, float>> candidatePosition = map.getCandidatePositions();
  map.emptyCandidatePositions();
  list<Pose> frontiers;
  vector<pair<float, float>>::iterator it = candidatePosition.begin();
  for (it; it != candidatePosition.end(); it++) {
    Pose p1 = Pose((*it).first, (*it).second, 0, range, FOV);
    Pose p2 = Pose((*it).first, (*it).second, 180, range, FOV);
    Pose p3 = Pose((*it).first, (*it).second, 90, range, FOV);
    Pose p4 = Pose((*it).first, (*it).second, 270, range, FOV);
    frontiers.push_back(p1);
    frontiers.push_back(p2);
    frontiers.push_back(p3);
    frontiers.push_back(p4);
  }
  EvaluationRecords *record = function->evaluateFrontiers(currentRobotWayPoint,
      frontiers, map, threshold, *path_client, *mapping_time_belief,
      *batteryTime, *belief_map, *mappingWaypoints, *tools, *distances_map);
  list<Pose> nearCandidates = record->getFrontiers();
  cout << "Number of candidates:" << nearCandidates.size() << endl;
  std::pair<string, list<Pose>> pair = make_pair(actualPose, nearCandidates);
  graph2->push_back(pair);
}

double Utilities::calculateScanTime(double scanAngle) {
  return (-7.2847174296449998e-006 * scanAngle * scanAngle * scanAngle +
          2.2131847908245512e-003 * scanAngle * scanAngle +
          1.5987873410233613e-001 * scanAngle + 10);
}

Pose Utilities::createFromInitialPose(Pose pose, float variation, int range,
                                      int FOV) {
  Pose tmp = Pose(pose.getX(), pose.getY(), pose.getOrientation() + variation,
                  FOV, range);
  return tmp;
}

void Utilities::calculateDistance(list<Pose> history,
                                  ros::ServiceClient *path_client,
                                  double robot_radius) {
  std::list<Pose>::iterator it = history.begin();
  double travelledDistance = 0;
  int numOfTurning = 0;
  nav_msgs::GetPlan path;
  path.request.goal.header.frame_id = "map";
  // Calculate the overall path connecting these cells
  for (it; it != prev(history.end(), 1); it++) {
    std::list<Pose>::iterator it2 = next(it, 1);
    path.request.goal.pose.position.x = it->getX();
    path.request.goal.pose.position.y = it->getY();
    path.request.goal.pose.orientation.w = 1;

    bool path_srv_call = path_client->call(path);
    float path_len;
    if (path_srv_call) {
      // calculate path length
      path_len = getPathLen(path.response.plan.poses, robot_radius);
      if (path_len < 1e3) {
      } else {
        path_len = 1000;
      }
    } else {
      cout
          << "[navigation_utilties.cpp@calculateDistance] Service call failed! "
          << endl;
      path_len = 1000;
    }
    travelledDistance = travelledDistance + path_len;
  }
  cout << "Number of cells: " << history.size() << endl;
  cout << "Num of Turning: " << numOfTurning << endl;
  cout << "Travelled distance (cells): " << travelledDistance << endl;
  cout << "Travelled distance (meters): " << travelledDistance / 2.0
       << endl; // Valid only if resolution == 1.0 (cell side is 0.5m)
}

void Utilities::updatePathMetrics(
    int *count, Pose *target, Pose *previous, string actualPose,
    list<Pose> *nearCandidates, vector<pair<string, list<Pose>>> *graph2,
    dummy::Map *map, MCDMFunction *function, list<Pose> *tabuList,
    list<pair<float, float>> *posToEsclude, vector<string> *history,
    int encodedKeyValue, long *numConfiguration, double *totalAngle,
    double *travelledDistance, int *numOfTurning, double scanAngle,
    ros::ServiceClient *path_client, bool backTracking, double robot_radius,
    unordered_map<string, string> *mappingWaypoints) {

  double path_len;
  // Add it to the list of visited cells as first-view
  history->push_back(function->getEncodedKey(*target, encodedKeyValue));
  //  cout << function->getEncodedKey ( *target,1 ) << endl;
  // Add it to the list of visited cells from which acting
  tabuList->push_back(*target);
  posToEsclude->push_back(
      make_pair(int((*target).getX()), int((*target).getY())));
  // Remove it from the list of candidate position
  cleanPossibleDestination2(nearCandidates, *target);
  // Push in the graph the previous robot pose and the new list of candidate
  // position, without the current pose of the robot
  // We don't want to visit this cell again
  if (backTracking == false) {
    std::pair<string, list<Pose>> pair = make_pair(actualPose, *nearCandidates);
    graph2->push_back(pair);
  }

  // Calculate the path from the previous robot pose to the current one

  // Gridmap
  // nav_msgs::GetPlan path;
  // path.request.start.header.frame_id = "map";
  // path.request.start.pose.position.x = previous->getX();
  // path.request.start.pose.position.y = previous->getY();
  // path.request.start.pose.orientation.w = 1;
  // path.request.goal.header.frame_id = "map";
  // path.request.goal.pose.position.x = target->getX();
  // path.request.goal.pose.position.y = target->getY();
  // path.request.goal.pose.orientation.w = 1;
  // bool path_srv_call = path_client->call(path);
  // if (path_srv_call) {
  //   // calculate path length
  //   path_len = getPathLen(path.response.plan.poses, robot_radius);
  // } else {
  //   cout << "[utils.cpp@updatePathMetrics] Path_finding Service call failed!
  //   " << endl;
  // }

  // Topological map
  // strands_navigation_msgs::GetRouteTo path;
  // string waypointName;
  // EvaluationRecords record;
  // bool found = false;
  // string encoding = record_.getEncodedKey(*target);
  // cout << "encoding: " << encoding << endl;
  // auto search = mappingWaypoints->find(encoding);
  // if (search != mappingWaypoints->end()) {
  //   waypointName = search->second;
  //   found = true;
  // } else {
  //   found = false;
  // }
  // cout << "waypointName: " << waypointName << endl;
  // if (found == true){
  //   path.request.goal = waypointName;
  //   bool path_srv_call  = path_client->call(path);
  //   if(path_srv_call){
  //     path_len = path.response.route.source.size();
  //   }else {
  //     cout << "[utils.cpp@updatePathMetrics] Path_finding Service call
  //     failed! " << endl;
  //   }
  // }else{
  //   path_len = 1000;
  // }

  // Update the distance counting
  // *travelledDistance = *travelledDistance + path_len;
  // Update the turning counting
  //  *numOfTurning = *numOfTurning + astar->getNumberOfTurning(path);
  // Update the scanning angle
  *totalAngle += scanAngle;
  // // Update the number of configurations of the robot along the task
  // (*numConfiguration)++;
  // // Update counter of iterations
  // (*count)++;
}

list<Pose> Utilities::cleanHistory(vector<string> *history,
                                   EvaluationRecords *record_history) {
  vector<string>::iterator it_history = history->begin();
  list<Pose> tmp_history;
  for (it_history; it_history != prev(history->end(), 1); it_history++) {
    if ((*it_history).back() == '1') {
      tmp_history.push_back(record_history->getPoseFromEncoding(*it_history));
    }
  }
  return tmp_history;
}

void Utilities::printResult(long newSensedCells, long totalFreeCells,
                            double precision, long numConfiguration,
                            double travelledDistance, int numOfTurning,
                            double totalAngle, double totalScanTime,
                            double resolution, float w_info_gain,
                            float w_travel_distance, float w_sensing_time,
                            float w_battery_status, float w_rfid_gain,
                            std::string fileURI) {
  cout << "-----------------------------------------------------------------"
       << endl;
  cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells << "[ "
       << 100 * float(newSensedCells) / float(totalFreeCells) << " %]" << endl;
  cout << "Total cell visited :" << numConfiguration << endl;
  cout << "Total travelled distance (meters): " << travelledDistance << endl;
  //  cout << "Total travel time: " << travelledDistance / resolution << "s, "
  //       << (travelledDistance / resolution) / 60 << " m" << endl;
  //  cout << "I came back to the original position since i don't have any other
  //  "
  //          "candidate position"
  //       << endl;
  //  cout << "Total exploration time (s): " << travelledDistance / resolution
  //  << endl; cout << "Total number of turning: " << numOfTurning << endl;
  cout << "Sum of scan angles (radians): " << totalAngle << endl;
  cout << "Total time for scanning: " << totalScanTime << endl;
  //  cout << "Total time for exploration: "
  //       << travelledDistance / resolution + totalScanTime << "s, "
  //       << (travelledDistance / resolution + totalScanTime) / 60 << " m" <<
  //       endl;
  if (newSensedCells < precision * totalFreeCells) {
    cout << "FINAL: MAP NOT EXPLORED! :(" << endl;
  } else {
    cout << "FINAL: MAP EXPLORED!" << endl;
  }
  std::ofstream txt(fileURI.c_str());
  txt << w_info_gain << "," << w_travel_distance << "," << w_sensing_time << ","
      << w_battery_status << "," << w_rfid_gain << ","
      << float(newSensedCells) / float(totalFreeCells) << ","
      << numConfiguration << "," << travelledDistance << "," << totalScanTime
      << endl;
  txt.close();
  cout << "-----------------------------------------------------------------"
       << endl;
}

Pose Utilities::getCurrentPose(float resolution, float costresolution,
                               dummy::Map *map, double initFov, int initRange) {
  ros::Time _now_stamp_ = ros::Time(0);

  tf::StampedTransform start_pose_in_tf;
  tf::TransformListener _tf_listener;
  string map_frame = "map";
  _tf_listener.waitForTransform(map_frame, "base_link", _now_stamp_,
                                ros::Duration(5.0));
  try {
    _tf_listener.lookupTransform(map_frame, "base_link", _now_stamp_,
                                 start_pose_in_tf);
  } catch (tf::TransformException &ex) {
    cout << "TRANSFORMS ARE COCKED-UP PAL! Why is that :=> " << ex.what()
         << endl;
  }

  tf::Vector3 start_position = start_pose_in_tf.getOrigin();
  //  cout << "Origin: " << start_position.x() << ", " << start_position.y() <<
  //  endl;
  tf::Quaternion start_orientation = start_pose_in_tf.getRotation();

  geometry_msgs::PoseStamped start_pose;
  start_pose.header.stamp = start_pose_in_tf.stamp_;
  start_pose.header.frame_id = start_pose_in_tf.frame_id_;

  tf::pointTFToMsg(start_position, start_pose.pose.position);
  tf::quaternionTFToMsg(start_orientation, start_pose.pose.orientation);

  float initX = roundf((start_pose.pose.position.x) * 100) / 100;
  float initY = roundf((start_pose.pose.position.y) * 100) / 100;
  tf::Quaternion quat = tf::Quaternion(
      start_pose.pose.orientation.x, start_pose.pose.orientation.y,
      start_pose.pose.orientation.z, start_pose.pose.orientation.w);
  tfScalar angle = roundf(2 * atan2(quat[2], quat[3]) * 100) / 100;

  // cout << "Current position in the " << map_frame << " frame:" << initX <<
  // "," << initY
  //      << " with orientation :" << angle << "(" << (angle * 180 / M_PI)
  //      << " deg)" << endl;

  int initOrientation = angle * 180 / M_PI;
  //  cout << "Orientation after casting: " << initOrientation << endl;

  // ATTENTION: should be adapted for cells different from 1mx1m
  // convert from map frame to image
  tf::Vector3 pose = tf::Vector3(initX, initY, 0.0);
  //  if (resolution >= 0 && resolution < 1 && resolution != costresolution)
  //  {
  //    //full resolution and scaling
  //    cout << "Full resolution and scaling" << endl;
  //    pose = pose / costresolution;
  ////    cout << "[BEFORE]Initial position in the Gazebo frame: " <<
  /// pose.getX() * costresolution<< "," << pose.getY() * costresolution <<
  /// endl;
  //    Pose initialPose = Pose(map->getPathPlanningNumRows() - (long)
  //    pose.getY() + costorigin.position.y / costresolution, (long) pose.getX()
  //    - costorigin.position.x / costresolution,
  //                            initOrientation, initRange, initFov);
  //    return initialPose;
  //  }
  //  else
  //  {
  // 1mx1m
  //    cout << endl << "1mx1m" << endl;
  // cout << "[BEFORE]Initial position in the image frame: " << pose.getX()<<
  // "," << map.getPathPlanningNumRows() - (long)pose.getY() << endl;
  // NOTE: Y in map are X in image
  Pose initialPose = Pose(pose.getX(), pose.getY(), angle, initRange, initFov);
  return initialPose;
  //  }
}

bool Utilities::move(float x, float y, float orientation, float time_travel,
                     list<Pose> *tabuList,
                     std::list<std::pair<float, float>> *posToEsclude) {
  move_base_msgs::MoveBaseGoal goal;
  bool success = false;

  MoveBaseClient ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(5.0))) {
    cout << "[navigation_utilities@createROSComms]... waiting ..." << endl;
  }
  // we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;

  double _orientation = orientation;
  // The orientation needs to be within [-M_PI, M_PI]
  if (orientation > M_PI)
    _orientation = orientation - 2 * M_PI;
  tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, _orientation);
  goal.target_pose.pose.orientation.x = q.getX();
  goal.target_pose.pose.orientation.z = q.getY();
  goal.target_pose.pose.orientation.z = q.getZ();
  goal.target_pose.pose.orientation.w = q.getW();

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  cout << "[navigation_utilties.cpp@move] Sending goal = (" << x << "," << y
       << ") with orientation: " << _orientation << "("
       << _orientation * 180 / M_PI << ")" << endl;
  ac.sendGoal(goal);
  //  cout << "[navigation_utilties.cpp@move] I'm moving..." << endl;

  actionlib::SimpleClientGoalState curr_state =
      actionlib::SimpleClientGoalState::PENDING;
  float total_wait_time = 0.0;
  while (!curr_state.isDone()) {
    time_travel = std::min(time_travel, (float)5.0);
    total_wait_time += time_travel;
    cout << "     [navigation_utilties.cpp@move] Waiting for "
         << total_wait_time << "/60.0 seconds to reach goal" << endl;
    ac.waitForResult(ros::Duration(time_travel));
    curr_state = ac.getState();
    cout << "     Result: " << curr_state.getText() << endl;

    if (curr_state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      cout << "[navigation_utilties.cpp@move] Goal position reached!" << endl;
      success = true;
    } else if (curr_state.isDone()) {
      cout << "[navigation_utilties.cpp@move] The base failed to move, adding "
              "this "
              "pose to the Tabulist and posToEsclude"
           << endl;
      success = false;
      std::pair<int, int> pairToRemove;
      pairToRemove = make_pair(int(x), int(y));
      posToEsclude->push_back(pairToRemove);
    } else if (total_wait_time > 60.0) {
      cout << "[navigation_utilties.cpp@move] Is taking too long" << endl;
      success = false;
      curr_state = actionlib::SimpleClientGoalState::ABORTED;
    }
  }
  return success;
}

double Utilities::getPathLen(std::vector<geometry_msgs::PoseStamped> poses,
                             double robot_radius) {
  double len = 0;
  geometry_msgs::Point p1, p2;
  int npoints = poses.size();
  //  cout << "[navigation_utilties.cpp@getPathLen]Path has [" <<npoints << "]
  //  points" << endl;
  if (npoints > 0) {
    for (int i = 1; i < npoints; i++) {
      p1 = poses[i].pose.position;
      p2 = poses[i - 1].pose.position;
      len += sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
      len += abs(getAngleBetPoses(poses[i], poses[i - 1])) * robot_radius;
    }
  } else {
    len = std::numeric_limits<double>::max();
    //    cout << "[navigation_utilties.cpp@getPathLen]Empty path. Len set to
    //    infinite... " << endl;
  }

  return len;
}
double Utilities::getAngleBetPoses(geometry_msgs::PoseStamped ps1,
                                   geometry_msgs::PoseStamped ps2) {

  // ....................
  // find quaternion to go from q1 to q2
  // q2 = qr *q1
  // qr = q2 * q1_inv

  double roll, pitch, yaw;
  // inverted quaternion is just inverting w component
  tf::Quaternion q1_inv(ps1.pose.orientation.x, ps1.pose.orientation.y,
                        ps1.pose.orientation.z, -ps1.pose.orientation.w);
  tf::Quaternion q2(ps2.pose.orientation.x, ps2.pose.orientation.y,
                    ps2.pose.orientation.z, ps2.pose.orientation.w);

  tf::Quaternion qr = q2 * q1_inv;
  qr.normalize();

  // and get angles
  tf::Matrix3x3 m(qr);
  m.getRPY(roll, pitch, yaw);
  //
  return yaw;
}

bool Utilities::showMarkerandNavigate(
    Pose target, ros::Publisher *marker_pub, dummy::Map *map,
    ros::ServiceClient *path_client, list<Pose> *tabuList,
    std::list<std::pair<float, float>> *posToEsclude, double min_robot_speed,
    double *batteryTime, double *travelledDistance,
    unordered_map<string, string> *mappingWaypoints, 
    strands_navigation_msgs::TopologicalMap topological_map,
    std::vector<string> tag_ids)
{
  //---------------------------PRINT GOAL POSITION
  // geometry_msgs::PointStamped p;
  // p.header.frame_id = "map";
  // p.header.stamp = ros::Time::now();
  // p.point.x = target.getX();
  // p.point.y = target.getY();

  goal_marker_.header.stamp = ros::Time::now();
  goal_marker_.pose.position.x = target.getX();
  goal_marker_.pose.position.y = target.getY();

  marker_pub->publish(goal_marker_);
  //----------------------------------------------
  // move_base_msgs::MoveBaseGoal goal;

  // Get distance from metric map
  // double path_len = criterion_utils_.computeMetricDistance(target, map, path_len);
  // Get distance from topological map
  double path_len = criterion_utils_.computeTopologicalDistance(target, path_client, mappingWaypoints);
  // cout << "   PathLen: " << path_len << endl;

  float time_travel = 2 * path_len / min_robot_speed;
  time_travel = std::max(time_travel, (float)180.0);
  *batteryTime -= time_travel;
  // cout << "   New batteryTime: " << *batteryTime << endl;
  *travelledDistance += path_len;

  return moveTopological(target, time_travel, tabuList, posToEsclude,
                         mappingWaypoints, topological_map, tag_ids, marker_pub);
}

bool Utilities::freeInLocalCostmap(
    Pose target, std::string move_base_local_costmap_topic_name) {
  string map_frame = "map";
  bool ans = false;
  int val = 0;
  grid_map::GridMap local_grid;
  geometry_msgs::PointStamped targetPoint_map;
  geometry_msgs::PointStamped targetPoint_local;

  targetPoint_map.point.x = target.getX();
  targetPoint_map.point.y = target.getY();
  targetPoint_map.header.frame_id = map_frame;

  // read a local costmap
  //      printf("............................................................
  //      \n");

  nav_msgs::OccupancyGridConstPtr local_costmap =
      ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(
          move_base_local_costmap_topic_name, ros::Duration(1.0));
  if (!local_costmap) {
    ROS_ERROR("Local occupancy grid not valid! \n");
  }
  //      printf("Local costmap frame is: %s
  //      \n",local_costmap->header.frame_id.c_str());

  // cast stupid point into local gridmap frame
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  geometry_msgs::TransformStamped map_to_local_tf;

  ros::Time _now_stamp_ = ros::Time(0);

  try {
    map_to_local_tf =
        tfBuffer.lookupTransform(local_costmap->header.frame_id, map_frame,
                                 _now_stamp_, ros::Duration(5.0));
  } catch (tf2::TransformException &ex) {
    cout << "TRANSFORMS ARE COCKED-UP PAL! Why is that :=> " << ex.what()
         << endl;
  }
  tf2::doTransform(targetPoint_map, targetPoint_local, map_to_local_tf);

  //      printf("In [%s] frame,  point is (%3.1f, %3.1f)
  //      \n",targetPoint_map.header.frame_id.c_str(), targetPoint_map.point.x,
  //      targetPoint_map.point.y); printf("In [%s] frame,  point is (%3.1f,
  //      %3.1f) \n",targetPoint_local.header.frame_id.c_str(),
  //      targetPoint_local.point.x, targetPoint_local.point.y);

  // cast occupancy grid to gridmap
  GridMapRosConverter::fromOccupancyGrid(*local_costmap, "layer", local_grid);

  grid_map::Position point(targetPoint_local.point.x,
                           targetPoint_local.point.y);
  // check target cell value inside  local costmap
  if (local_grid.isInside(point)) {
    //        printf("Point is INSIDE \n");
    // profit
    val = local_grid.atPosition("layer", point);
    //        printf("Value was %d \n",val);
  } else {
    //        printf("Point is OUTSIDE ... \n");
  }

  ans = (val == 0);
  //      printf("............................................................
  //      \n");
  return ans;
}

Pose Utilities::selectFreePoseInLocalCostmap(string currentRobotWayPoint, 
    Pose target, list<Pose> *nearCandidates, dummy::Map *map,
    MCDMFunction *function, double threshold, ros::ServiceClient *path_client,
    vector<unordered_map<float,
                         std::pair<string, bayesian_topological_localisation::
                                               DistributionStamped>>>
        *mapping_time_belief,
    std::list<std::pair<float, float>> *posToEsclude, EvaluationRecords *record,
    std::string move_base_local_costmap_topic_name, double *batteryTime,
    GridMap *belief_map, unordered_map<string, string> *mappingWaypoints,
    prediction_tools *tools, std::unordered_map<string, double> *distances_map) {
  bool isFreeFromObstacle = false;
  while (isFreeFromObstacle == false) {
    //    cout << "===> Checking against localmap!" << endl;
    // check that the current target is free from obstacles in the local map
    isFreeFromObstacle =
        freeInLocalCostmap(target, move_base_local_costmap_topic_name);
    // cout << "   isFree: " << isFreeFromObstacle << endl;
    if (isFreeFromObstacle == true)
      break; // if it's free, the while will be break immediately
    // Otherwise you have to select a new target and continue in the while
    // 1) Add this cell to the list of cell not reachebale
    std::pair<int, int> pairToRemove = make_pair(target.getX(), target.getY());
    posToEsclude->push_back(pairToRemove);
    // remove the current target from the list of candidate position
    // cout << "nearCandidate before: " << nearCandidates->size() << endl;
    cleanPossibleDestination2(nearCandidates, target);
    // cout << "nearCandidate after: " << nearCandidates->size() << endl;
    // Get the list of new candidate position with  associated evaluation
    record = function->evaluateFrontiers(currentRobotWayPoint,
        *nearCandidates, *map, threshold, *path_client, *mapping_time_belief,
        *batteryTime, *belief_map, *mappingWaypoints, *tools, *distances_map);
    // Get a new target
    std::pair<Pose, double> result = function->selectNewPose(record);
    //    cout << "     record size: " << record->size() << endl;
    target = result.first;
    //  cout << "     New target selected: " << target.getX() << ", " <<
    //  target.getY() << endl;
    // and start the new iteration
  }
  return target;
}

void Utilities::filePutContents(const std::string &name,
                                const std::string &content, bool append) {
  std::ofstream outfile;
  std::ifstream pFile(name);
  if (outfile.fail()) {
    std::cout << "Error while opening the stream." << endl;
    // std::cout << "File does not exist! Create a new one!" << endl;
    // outfile.open(name);
    // outfile <<
    // "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,coverage,numConfiguration,travelledDistance,totalScanTime";
  } else {
    if (pFile.peek() == std::ifstream::traits_type::eof()) { // file is empty
      // std::cout << "File does not exist! Create a new one!" << endl;
      outfile.open(name);
      if (name.find("result") != string::npos) {
        outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,w_"
                   "battery_status,norm_w_info_gain,norm_w_travel_distance,"
                   "norm_w_sensing_time,norm_w_rfid_gain,norm_w_battery_status,"
                   "coverage,numConfiguration,travelledDistance,totalScanTime,"
                   "accumulatedRxPower,batteryStatus,accuracy"
                << endl;
      } else if (name.find("coverage") != string::npos) {
        outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,w_"
                   "battery_status,norm_w_info_gain,norm_w_travel_distance,"
                   "norm_w_sensing_time,norm_w_rfid_gain,norm_w_battery_status,"
                   "numConfiguration,increasingCoverage,travelledDistance"
                << endl;
      } else if (name.find("distance") != string::npos) {
        outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,w_"
                   "battery_status,tag1,tag2,tag3,tag4,tag5,tag6,tag7,tag8,"
                   "tag9,tag10"
                << endl;
      } else if (name.find("accuracy") != string::npos) {
        outfile << "w_info_gain,w_travel_distance,w_sensing_time,w_rfid_gain,w_"
                   "battery_status,range,numConfiguration,accuracy"
                << endl;
      } else if (name.find("prediction") != string::npos) {
        outfile << "pf_prediction, gt_closer_waypoint, metric_distance" << endl;
      } else if (name.find("gt_tag_pose") != string::npos) {
        outfile << "gt_x, gt_y" << endl;
      } else if (name.find("pf_tag_pose") != string::npos) {
        outfile << "pf_x, pf_y" << endl;
      } else if (name.find("gps_tag_pose") != string::npos) {
        outfile << "gps_x, gps_y" << endl;
      }
    } else {
      // std::cout << "File exists! Appending data!" << endl;
      outfile.open(name, std::ios_base::app);
    }
  }
  outfile << content;
}

std::vector<std::pair<int, std::pair<int, int>>>
Utilities::findTagFromBeliefMap(GridMap *belief_map) {

  std::vector<std::pair<int, std::pair<int, int>>> tag_positions;

  // First, remove the useless layers
  std::vector<string> layers_name = belief_map->getLayers();
  // The layers_name vector contains "ref_map, X, Y" which are for not for
  // finding the tags. So we can remove their name to avoid checking this
  // layers.
  layers_name.erase(layers_name.begin(), layers_name.begin() + 3);
  for (auto it = layers_name.begin(); it != layers_name.end(); it++) {
    std::pair<int, int> tag(0, 0);
    double likelihood = 0;
    // Access the belief map of every tag
    GridMap::Matrix &grid = (*belief_map)[*it];
    // Iterate over a submap and sum the likelihood in all the cells
    for (GridMapIterator iterator(*belief_map); !iterator.isPastEnd();
         ++iterator) {
      const grid_map::Index index(*iterator);
      // For every cell, analyse the surrounding area
      double tmp_likelihood = grid(index(0), index(1));
      int buffer_size = 3;
      if (index(0) > buffer_size and
          index(0) <= belief_map->getLength().x() - buffer_size) {
        if (index(1) > buffer_size and
            index(1) <= belief_map->getLength().y() - buffer_size) {
          // std::cout << "I: " << index(0) << "," << index(1) << endl;
          grid_map::Index submapStartIndex(index(0) - buffer_size,
                                           index(1) - buffer_size);
          grid_map::Index submapBufferSize(buffer_size, buffer_size);
          for (grid_map::SubmapIterator sub_iterator(
                   *belief_map, submapStartIndex, submapBufferSize);
               !sub_iterator.isPastEnd(); ++sub_iterator) {
            grid_map::Index sub_index(*sub_iterator);
            // std::cout << "I: " << sub_index(0) << "," << sub_index(1) <<
            // endl;
            tmp_likelihood += grid(sub_index(0), sub_index(1));
          }
        }
      }

      if (tmp_likelihood > likelihood) {
        likelihood = tmp_likelihood;
        Position tag_p;
        belief_map->getPosition(index, tag_p);
        tag.first = tag_p.x();
        tag.second = tag_p.y();
      }
    }
    std::pair<int, std::pair<int, int>> single_tag(likelihood, tag);
    tag_positions.push_back(single_tag);
  }

  return tag_positions;
}

void Utilities::convertStrandTopoMapToListPose(
    strands_navigation_msgs::TopologicalMap *topoMap, list<Pose> *frontiers,
    int range, double FoV, unordered_map<string, string> *mappingWaypoints) {
  tf::Quaternion quat;
  tfScalar angle;
  Pose tmpPose;
  string encoding;
  // Iterate on all the nodes of the topological map and create a list of Pose
  // elements
  for (auto nodes_it = topoMap->nodes.begin(); nodes_it != topoMap->nodes.end();
       nodes_it++) {
    quat = tf::Quaternion(
        nodes_it->pose.orientation.x, nodes_it->pose.orientation.y,
        nodes_it->pose.orientation.z, nodes_it->pose.orientation.w);

    // NOTE: create eight pose for every node
    for (int i = 0; i < 1; i++) {
      tmpPose =
          Pose(nodes_it->pose.position.x, nodes_it->pose.position.y,
               // roundf(i* M_PI / 4 * 100) / 100,
               // angle * (-1 + i*2), // only two angles, one defined by the
               // topology and the opposite one
               angle * (1 - i) + (i)*fmod(angle + M_PI, 2 * M_PI), range, FoV);
      frontiers->push_back(tmpPose);
      encoding = record_.getEncodedKey(tmpPose);
      mappingWaypoints->insert(std::make_pair(encoding, nodes_it->name));
    }

    // NOTE: one note per node, like in the topological map
    // angle = roundf(2 * atan2(quat[2], quat[3]) * 100) / 100;
    // tmpPose = Pose(nodes_it->pose.position.x,
    //                 nodes_it->pose.position.y,
    //                 angle,
    //                 range,
    //                 FoV);
    // frontiers->push_back(tmpPose);
    // encoding = record_.getEncodedKey(tmpPose);
    // mappingWaypoints->insert(std::make_pair(encoding, nodes_it->name));
  }
  // cout <<"[TOPOMAP]: " << frontiers->size() << " nodes" << endl;
}

void Utilities::setGazeboModelStateClient(ros::ServiceClient &gazebo_model_state_client)
{
  this->gazebo_model_state_client = &gazebo_model_state_client;
}

bool Utilities::getGazeboModelPose(string model_name, string relative_entity_name, geometry_msgs::Pose *model_pose)
{
  // Create srv request for gazebo model
  gazebo_msgs::GetModelState model_state_srv;
  if (!this->gazebo_model_state_client){
    std::cout << "[utils.cpp@getGazeboModelPose]... set the gazebo client first with setGazeboModelStateClient" << std::endl;
    return false;
  }
  model_state_srv.request.model_name = model_name;
  model_state_srv.request.relative_entity_name = relative_entity_name;
  if (this->gazebo_model_state_client->call(model_state_srv))
  {
    *model_pose = model_state_srv.response.pose;
    // cout << "Got model pose for " << model_name << endl;
    return true;
  }
  return false;
}

bool Utilities::getModelClosestWaypoint(
    string model_name, strands_navigation_msgs::TopologicalMap topological_map,
    string &closest_waypoint_name, geometry_msgs::Pose &closest_waypoint_pose)
{
  // string model_name = "tag_" + tag_id;
  string relative_entity_name = "map";

  if (getGazeboModelPose(model_name, relative_entity_name, &closest_waypoint_pose))
  {
    // Look for closer waypoint to current pose
    // and compare it to the PF prediction
    closest_waypoint_name = getCloserWaypoint(closest_waypoint_pose, topological_map);
    // cout << "closest_wp_name " << closest_waypoint_name << endl;
    return true;
  }
  return false;
}

void Utilities::checkOnNode(strands_navigation_msgs::TopologicalMap topological_map)
{
  actionlib::SimpleClientGoalState curr_state = actionlib::SimpleClientGoalState::PENDING;
  topological_navigation::GotoNodeActionGoal topoGoal;
  float total_wait_time = 0.0;
  float sleep_time = 2.0;
  std_msgs::StringConstPtr current_node = ros::topic::waitForMessage<std_msgs::String>("/current_node", ros::Duration(5));
  if (current_node && current_node->data == "none")
  {
    string closest_wp;
    geometry_msgs::Pose closest_waypoint_pose;
    if (getModelClosestWaypoint("thorvald_ii", topological_map, closest_wp, closest_waypoint_pose))
    {
      topoGoal.goal.target = closest_wp;
      topoGoal.goal.no_orientation = true;
      cout << "[utils.cpp@moveTopological] Going temporarily to the closest node: "
           << topoGoal.goal.target << endl;
      this->topoAC->sendGoal(topoGoal.goal);
      while (!curr_state.isDone() && total_wait_time < 10.0)
      {
        total_wait_time += sleep_time;
        this->topoAC->waitForResult(ros::Duration(sleep_time));
        curr_state = this->topoAC->getState();
      }
    }
    else
    {
      cout << "[utils.cpp@moveTopological] No closest node found for thorvald_ii" << endl;
    }
  }
}

bool Utilities::moveTopological(
    Pose target, float time_travel, list<Pose> *tabuList,
    std::list<std::pair<float, float>> *posToEsclude,
    unordered_map<string, string> *mappingWaypoints,
    strands_navigation_msgs::TopologicalMap topological_map,
    std::vector<string> tag_ids, ros::Publisher *marker_pub)
{

  topological_navigation::GotoNodeActionGoal topoGoal;
  
  string encoding = record_.getEncodedKey(target);
  bool success = false;

  if (!this->topoAC) {

    this->topoAC = new actionlib::SimpleActionClient<topological_navigation::GotoNodeAction>("topological_navigation", true);
  }
  
  while (!this->topoAC->waitForServer(ros::Duration(5.0))) {
    cout << "[utils.cpp@moveTopological]... waiting ..." << endl;
  }

  topoGoal.header.frame_id = "map";
  topoGoal.header.stamp = ros::Time::now();

  // check if the robot is outside nodes influence  
  checkOnNode(topological_map); // this is blocking; goes to the closest node if not on it

  auto search = mappingWaypoints->find(encoding);
  string waypointName;
  if (search != mappingWaypoints->end()) {
    waypointName = search->second;
    // std::cout << "Found :" << search->second << '\n';
  } else {
    std::cout << "[utils.cpp@moveTopological]Not found\n";
  }
  topoGoal.goal.target = waypointName;
  topoGoal.goal.no_orientation = false;

  // NOTE: since time passes between pose selection and sending goal, it can be
  // that the target is now occupied by the tag. We need then to select a
  // connected node to the one we would like to go.
  bool change_destination = updateDestinationWaypoint(
      tag_ids, topological_map, waypointName, target, *marker_pub, topoGoal, success, posToEsclude);

  cout << "[utils.cpp@moveTopological] New robot destination: "
             << topoGoal.goal.target << endl;
  this->topoAC->sendGoal(topoGoal.goal);

  actionlib::SimpleClientGoalState curr_state = actionlib::SimpleClientGoalState::PENDING;
  float total_wait_time = 0.0;
  float sleep_time = 2.0;
  if (time_travel < 180.0) time_travel = 180.0;  // This should be useless
  while (!curr_state.isDone()) {
    total_wait_time += sleep_time;
    this->topoAC->waitForResult(ros::Duration(sleep_time));
    curr_state = this->topoAC->getState();
        
    if (curr_state == actionlib::SimpleClientGoalState::SUCCEEDED && this->topoAC->getResult()->success) {
      cout << "[utils.cpp@moveTopological] Goal position reached!" << endl;
      success = true;
    } else if (curr_state.isDone()) {
      cout << "[utils.cpp@moveTopological] The base failed to move" << endl; 
      success = false;
    } else if (total_wait_time > time_travel) {
      cout << "[utils.cpp@moveTopological] Is taking too long" << endl;
      success = false;
      this->topoAC->cancelAllGoals();
      curr_state = actionlib::SimpleClientGoalState::ABORTED;
    }else { // here we are still navigating toward the goal
      // check that the goal is still reachable, i.e. that no tag is on it
      // cout << "   ... checking robot and tag positions..." << endl;
      change_destination = updateDestinationWaypoint(
        tag_ids, topological_map, waypointName, target, *marker_pub, topoGoal, success, posToEsclude);
      if (change_destination == true){
        this->topoAC->cancelAllGoals();
        // check if the robot is outside nodes influence
        checkOnNode(topological_map); // this is blocking; goes to the closest node if not on it
        // Send the updated goal
        this->topoAC->sendGoal(topoGoal.goal);
        curr_state = actionlib::SimpleClientGoalState::PENDING;
        cout << "[utils.cpp@moveTopological] Destination have been changed again: "
             << topoGoal.goal.target << endl;
      }

      // for (auto tag_id : tag_ids) {
      //   if (getTagClosestWaypoint(tag_id, topological_map, closest_waypoint_name_, closest_waypoint_pose_)){
      //     if (closest_waypoint_name_ == waypointName){
      //       //abort navigation
      //       cout << "[utils.cpp@moveTopological] We are already navigating but an agent is on the goal node." << endl;
      //       success = false;
      //       this->topoAC->cancelAllGoals();
      //       auto [new_waypointName, new_waypointPose] = getCloserConnectedWaypoint(waypointName, target, &topological_map);
      //       topoGoal.goal.target = new_waypointName;
      //       topoGoal.goal.no_orientation = false;
      //       this->topoAC->sendGoal(topoGoal.goal);
      //       curr_state = actionlib::SimpleClientGoalState::PENDING;
      //       cout << "[utils.cpp@moveTopological] ... now going to: " << new_waypointName << endl;
      //       //---------------------------PRINT GOAL POSITION
      //       // geometry_msgs::PointStamped p;
      //       // p.header.frame_id = "map";
      //       // p.header.stamp = ros::Time::now();
      //       // p.point.x = new_waypointPose.position.x;
      //       // p.point.y = new_waypointPose.position.y;
      //       goal_marker_.header.stamp = ros::Time::now();
      //       goal_marker_.pose.position.x = new_waypointPose.position.x;
      //       goal_marker_.pose.position.y = new_waypointPose.position.y;
      //       marker_pub->publish(goal_marker_);
      //       //----------------------------------------------
      //       waypointName = new_waypointName;
      //     }
      //   }
      // }
    }
  }
  return success;
}

bayesian_topological_localisation::DistributionStamped
Utilities::convertGridBeliefMapToTopoMap(
    GridMap belief_map, list<Pose> topoMap,
    unordered_map<string, string> mappingWaypoints, string tag_id, double radius) {
  // Retrieve node waypoint name from the mapping and return waypoint and summed
  // belief inside the message

  string encoding, waypointName;
  double probability;
  bayesian_topological_localisation::DistributionStamped topo_belief;
  // grid_map::Matrix& data = (*belief_map)[tag_id];


  for (auto it = topoMap.begin(); it != topoMap.end(); it++) {
    probability = 0.0;
    encoding = record_.getEncodedKey(*it);
    auto search = mappingWaypoints.find(encoding);
    if (search != mappingWaypoints.end()) {
      waypointName = search->second;
    } else {
      std::cout
          << "[convertGridBeliefMapToTopoMap@utils.cpp] Encoding not found\n";
    }

    Position center(it->getX(), it->getY());
    Position point;
    for (grid_map::CircleIterator iterator(belief_map, center, radius);
        !iterator.isPastEnd(); ++iterator) {
     // const grid_map::Index index(*iterator);
     // probability = probability + data((index(0), index(1)));
     belief_map.getPosition(*iterator, point);
     probability += belief_map.atPosition(tag_id, point);
    }
    // Add the information to the returned object
    topo_belief.nodes.push_back(waypointName);
    topo_belief.values.push_back(probability);
  }
  return topo_belief;
}

string
Utilities::getCloserWaypoint(geometry_msgs::Pose &pose,
                             strands_navigation_msgs::TopologicalMap topoMap) {
  float minDistance = 100;
  float tmpDistance = 0;
  string closerWaypoint;

  for (auto nodesIt = topoMap.nodes.begin(); nodesIt != topoMap.nodes.end();
       nodesIt++) {
    tmpDistance = sqrt(pow(pose.position.x - nodesIt->pose.position.x, 2) +
                       pow(pose.position.y - nodesIt->pose.position.y, 2));
    if (tmpDistance < minDistance) {
      minDistance = tmpDistance;
      closerWaypoint = nodesIt->name;
    }
  }
  return closerWaypoint;
}

tuple <string, geometry_msgs::Pose>
Utilities::getCloserConnectedWaypoint(string node_name, Pose pose,
        strands_navigation_msgs::TopologicalMap *topoMap, std::list<std::pair<float, float>> *posToEsclude)
{
  float minDistance = 100;
  float tmpDistance = 0;
  // hardcoded, sigh :(
  int range = 5;
  double FOV = 180;
  string closerWaypoint;
  geometry_msgs::Pose closerPose;
  std::vector<strands_navigation_msgs::Edge> edges;
  std::pair<float, float> tmp_pose;

  for (auto nodesIt = topoMap->nodes.begin(); nodesIt != topoMap->nodes.end();
       nodesIt++)
  {
    if (nodesIt->name.compare(node_name) == 0){
      edges = nodesIt->edges;
      break;
    }

  }
  
  do{
    for (auto edgesIt = edges.begin(); edgesIt != edges.end();
          edgesIt++)
    {
      for (auto nodesIt = topoMap->nodes.begin(); nodesIt != topoMap->nodes.end();
            nodesIt++)
      {
        if (nodesIt->name.compare(edgesIt->node) == 0) 
        {
          tmpDistance = sqrt(pow(pose.getX() - nodesIt->pose.position.x, 2) +
                                pow(pose.getY() - nodesIt->pose.position.y, 2));
          if (tmpDistance < minDistance)
          {
            minDistance = tmpDistance;
            closerWaypoint = nodesIt->name;
            closerPose.position.x = nodesIt->pose.position.x;
            closerPose.position.y = nodesIt->pose.position.y;
            closerPose.orientation.w = 1.0;
          }
        }  
      }
    }
    tmp_pose.first = closerPose.position.x;
    tmp_pose.second = closerPose.position.y;
  }while (!this->containsPos(posToEsclude, tmp_pose));
  return {closerWaypoint, closerPose};
}

geometry_msgs::Pose Utilities::getWaypointPoseFromName(
    string name, strands_navigation_msgs::TopologicalMap *topoMap) {
  for (auto nodesIt = topoMap->nodes.begin(); nodesIt != topoMap->nodes.end();
       nodesIt++) {
    if (nodesIt->name.compare(name) == 0)
      return nodesIt->pose;
  }
  cout << "[utils.cpp@getWaypointPoseFromName] Waypoint NOT FOUND!" << endl;
}

vector<unordered_map<
    float,
    std::pair<string, bayesian_topological_localisation::DistributionStamped>>>
Utilities::getStatelessRFIDBelief(
    double secs_from_now, bool return_history,
    vector<ros::ServiceClient> *pf_stateless_likelihoodClient_list) {
  vector<unordered_map<
      float, std::pair<string,
                       bayesian_topological_localisation::DistributionStamped>>>
      mapping_time_belief;
  bayesian_topological_localisation::Predict prediction_stateless_srv;
  prediction_stateless_srv.request.secs_from_now = secs_from_now;
  prediction_stateless_srv.request.return_history = return_history;
  prediction_stateless_srv.request.prediction_rate = 0.1; // a prediction every 10 seconds....to approximate
  mapping_time_belief.clear(); // Remove old belief
  for (int tag_index = 0;
       tag_index < pf_stateless_likelihoodClient_list->size(); tag_index++) {
    if (pf_stateless_likelihoodClient_list->at(tag_index).call(
            prediction_stateless_srv)) {
      vector<double> timestamp =
          prediction_stateless_srv.response.secs_from_now;
      vector<string> estimated_node =
          prediction_stateless_srv.response.estimated_node;
      vector<bayesian_topological_localisation::DistributionStamped> prob_dist =
          prediction_stateless_srv.response.prob_dist;
      unordered_map<float, std::pair<string, bayesian_topological_localisation::
                                                 DistributionStamped>>
          map;
      cout << "Got stateless prediction for tag " << tag_index << endl;
      for (int ts_counter = 0; ts_counter < timestamp.size(); ts_counter++) {
        map.emplace(std::make_pair(
            timestamp[ts_counter],
            std::make_pair(estimated_node[ts_counter], prob_dist[ts_counter])));
      }
      mapping_time_belief.push_back(map);

    } else
      cout << "Stateless Srv No answer" << endl;
  }
  return mapping_time_belief;
}


bool Utilities::updateDestinationWaypoint(std::vector<string> tag_ids, 
            strands_navigation_msgs::TopologicalMap topological_map, string &waypointName,
            Pose &target, ros::Publisher &marker_pub,
            topological_navigation::GotoNodeActionGoal &topoGoal,
            bool &success, std::list<std::pair<float, float>> *posToEsclude){
  bool change_destination = false;
  for (auto tag_id : tag_ids) {
    string model_name = "tag_" + tag_id;
    // cout << "Model: " << model_name << endl;
    string closest_waypoint_name_;
    geometry_msgs::Pose closest_waypoint_pose_;
    if (getModelClosestWaypoint(model_name, topological_map, closest_waypoint_name_,
                              closest_waypoint_pose_)) {
      // cout << "     closest_waypoint_name_: " << closest_waypoint_name_ << endl;
      // cout << "     wayPointName: " << waypointName << endl;
      if (closest_waypoint_name_ == waypointName) {
        cout << "[utils.cpp@updateDestinationWaypoint] An agent is "
                "on the goal node. Changing destination to neighbour node."
             << endl;
        // Go to the connected node which is closest to the robot (to avoid robot navigating too much)
        geometry_msgs::Pose robot_pose;
        getGazeboModelPose("thorvald_ii", "map", &robot_pose);
        Pose robot_target = Pose(robot_pose.position.x, robot_pose.position.y, 0, 0, 0);
        auto [new_waypointName, new_waypointPose] =
            getCloserConnectedWaypoint(waypointName, robot_target, &topological_map, posToEsclude);
        topoGoal.goal.target = new_waypointName;
        topoGoal.goal.no_orientation = false;
        goal_marker_.header.stamp = ros::Time::now();
        goal_marker_.pose.position.x = new_waypointPose.position.x;
        goal_marker_.pose.position.y = new_waypointPose.position.y;
        marker_pub.publish(goal_marker_);
        waypointName = new_waypointName;
        change_destination = true;
        success = false;
      }
    }
  }
  return change_destination;
}

void Utilities::saveMap(const std::unordered_map<string, double> *map, string path){
  std::cout<<"Save in " << path << endl;
  const auto start = std::chrono::steady_clock::now();
  std::ofstream f;
  f.open(path.c_str());
  for(auto iter=map->begin(); iter!=map->end(); iter++){
      // fwrite(&(iter->first), sizeof(string), 1, f);
      // fwrite(&(iter->second), sizeof(double), 1, f);
      f << iter->first << "," << iter->second << "\n" ;
  }
  f.close();
  const auto end = std::chrono::steady_clock::now();
  std::cout<<"Save time = "<< std::chrono::duration<double, std::milli> (end-start).count() << " ms" << std::endl;
}

bool Utilities::loadMap(std::unordered_map<string, double> *map, string path){
  std::cout<<"Load from " << path << endl;
  const auto start = std::chrono::steady_clock::now();
  std::ifstream f;
  f.open(path.c_str());
  if (!f.good()) { 
    return false; 
  };

  string key;
  string val;
  // cout << "here" << endl;
  // while(fread(&key, sizeof(string), 1, f)){
  //     fread(&val, sizeof(double), 1, f);
  //   // cout << "Key: " << *key << ", val :" << *val << endl;
  //     // tmp_map.emplace(*key, *val);
  //     map->at(key) = val;
  //   // cout << "3" << endl;
  // }
  // fclose(f);
  while(f.good()){
    getline(f, key, ',');
    getline(f, val);
    map->emplace(key, std::atof(val.c_str()) );
  }
  const auto end = std::chrono::steady_clock::now();
  std::cout<<"Load time = "<< std::chrono::duration<double, std::milli> (end-start).count() << " ms" << std::endl;
  return true;
}
