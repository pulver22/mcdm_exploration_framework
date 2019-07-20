//
// Created by pulver on 20/07/19.
//

#include "navigation_utilities.h"

NavigationUtilities::NavigationUtilities() {};
NavigationUtilities::~NavigationUtilities() {};


bool NavigationUtilities::contains(std::list<Pose> &list, Pose &p) {
  bool result = false;
  MCDMFunction function;

  std::list<Pose>::iterator findIter = std::find(list.begin(), list.end(), p);
  if (findIter != list.end()) {
    // cout << "Found it: "<< function.getEncodedKey(p,0) <<endl;
    result = true;
  }
  return result;
}

bool NavigationUtilities::containsPos(std::list<std::pair<float, float>> *positionEscluded,
                 std::pair<float, float> p) {
  bool result = false;
  std::pair<float, float> tmp_p = make_pair(float(int(p.first)), float(int(p.second)));
//  cout << "   [pure_navigation.py@containsPos] tmp_p: " << tmp_p.first << "," << tmp_p.second << endl;

  auto findIter = std::find(positionEscluded->begin(), positionEscluded->end(), tmp_p);
  if (findIter != positionEscluded->end()) {
    result = true;
  }
//  cout << "Is it already visited and excluded: " << result << endl;
  return result;
}

void NavigationUtilities::cleanPossibleDestination2(std::list<Pose> *possibleDestinations, Pose &p) {
  MCDMFunction function;
  //  cout << "[pure_navigation.cpp@cleanPossibleDestination2] I want to remove
  //  "<< function.getEncodedKey(p,0) << endl;
  //  cout << "[pure_navigation.cpp@cleanPossibleDestination2] Size possible
  //  destination: " << possibleDestinations->size() << endl;
  //  for (auto it = possibleDestinations->begin(); it !=
  //  possibleDestinations->end(); it++)
  //  {
  //    cout << function.getEncodedKey(*it, 0) << endl;
  //  }
  std::list<Pose>::iterator findIter =
      std::find(possibleDestinations->begin(), possibleDestinations->end(), p);
  if (findIter != possibleDestinations->end()) {
    //    cout << "[pure_navigation.cpp@cleanPossibleDestination2] EncodedKey:"
    //    <<  function.getEncodedKey(*findIter,0) << "\n" << endl;
    possibleDestinations->erase(findIter);
  }
  //  else cout<< "[pure_navigation.cpp@cleanPossibleDestination2] Cell not
  //  found\n" << endl;

  //  for (auto it = possibleDestinations->begin(); it !=
  //  possibleDestinations->end(); it++)
  //  {
  //    cout << function.getEncodedKey(*it, 0) << endl;
  //  }
}

void NavigationUtilities::pushInitialPositions(dummy::Map map, float x, float y, float orientation,
                          int range, int FOV, double threshold,
                          string actualPose,
                          vector<pair<string, list<Pose> >> *graph2,
                          ros::ServiceClient *path_client) {
  MCDMFunction function;
  map.findCandidatePositions(x, y, orientation, FOV, range);
  vector<pair<float, float> > candidatePosition = map.getCandidatePositions();
  map.emptyCandidatePositions();
  list<Pose> frontiers;
  vector<pair<float, float> >::iterator it = candidatePosition.begin();
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
  EvaluationRecords *record =
      function.evaluateFrontiers(frontiers, &map, threshold, path_client);
  list<Pose> nearCandidates = record->getFrontiers();
  cout << "Number of candidates:" << nearCandidates.size() << endl;
  std::pair<string, list<Pose> > pair = make_pair(actualPose, nearCandidates);
  graph2->push_back(pair);
}

double NavigationUtilities::calculateScanTime(double scanAngle) {
  return (-7.2847174296449998e-006 * scanAngle * scanAngle * scanAngle +
          2.2131847908245512e-003 * scanAngle * scanAngle +
          1.5987873410233613e-001 * scanAngle + 10);
}

Pose NavigationUtilities::createFromInitialPose(Pose pose, float variation, int range, int FOV) {
  Pose tmp = Pose(pose.getX(), pose.getY(), pose.getOrientation() + variation,
                  FOV, range);
  return tmp;
}

void NavigationUtilities::calculateDistance(list<Pose> history, ros::ServiceClient *path_client, double robot_radius) {
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
      cout << "[pure_navigation.cpp@calculateDistance] Service call failed! " << endl;
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

void NavigationUtilities::updatePathMetrics(
    int *count, Pose *target, Pose *previous, string actualPose,
    list<Pose> *nearCandidates, vector<pair<string, list<Pose> >> *graph2,
    dummy::Map *map, MCDMFunction *function, list<Pose> *tabuList,
    list<pair<float, float> > *posToEsclude, vector<string> *history,
    int encodedKeyValue, long *numConfiguration,
    double *totalAngle, double *travelledDistance, int *numOfTurning,
    double scanAngle, ros::ServiceClient *path_client, bool backTracking, double robot_radius) {

  nav_msgs::GetPlan path;
  double path_len;
  // Add it to the list of visited cells as first-view
  history->push_back(function->getEncodedKey(*target, encodedKeyValue));
  //  cout << function->getEncodedKey ( *target,1 ) << endl;
  // Add it to the list of visited cells from which acting
  tabuList->push_back(*target);
//  posToEsclude->push_back(make_pair(roundf(((*target).getX() * 100) / 100),
//                                    roundf((*target).getY() * 100 / 100)));
  posToEsclude->push_back(make_pair(int((*target).getX()), int((*target).getY())));
  // Remove it from the list of candidate position
  cleanPossibleDestination2(nearCandidates, *target);
  // Push in the graph the previous robot pose and the new list of candidate
  // position, without the current pose of the robot
  // We don't want to visit this cell again
  if (backTracking == false) {
    std::pair<string, list<Pose> > pair = make_pair(actualPose, *nearCandidates);
    graph2->push_back(pair);
  }

  // Calculate the path from the previous robot pose to the current one
  //  string path = astar->pathFind ( target->getX(), target->getY(),
  //  previous->getX(), previous->getY(), map );
  path.request.start.header.frame_id = "map";
  path.request.start.pose.position.x = previous->getX();
  path.request.start.pose.position.y = previous->getY();
  path.request.start.pose.orientation.w = 1;
  path.request.goal.header.frame_id = "map";
  path.request.goal.pose.position.x = target->getX();
  path.request.goal.pose.position.y = target->getY();
  path.request.goal.pose.orientation.w = 1;
  //  cout << " (x_start, y_start) = (" << previous->getX() << "," <<
  //  previous->getY() << "), (x_goal, y_goal) = (" <<
  //          target->getX() << "," << target->getY() << ")" << endl;
  bool path_srv_call = path_client->call(path);
  if (path_srv_call) {
    // calculate path length
    path_len = getPathLen(path.response.plan.poses, robot_radius);
    //    if (path_len<1e3)
    //    {
    //      printf("Path len is [%3.3f m.]",path_len);
    //    }
    //    else
    //    {
    //      printf("Path len is infinite");
    //      path_len = 1000;
    //    }
  } else {
    cout << "[pure_navigation@updatePathMetrics] Path_finding Service call failed! " << endl;
  }
//    cout << "1: " << *travelledDistance << endl;
  // Update the distance counting
  *travelledDistance = *travelledDistance + path_len;
//    cout << "2: " << *travelledDistance << endl;
  // Update the turning counting
  //  *numOfTurning = *numOfTurning + astar->getNumberOfTurning(path);
  // Update the scanning angle
  *totalAngle += scanAngle;
  // Update the number of configurations of the robot along the task
  (*numConfiguration)++;
  // Update counter of iterations
  (*count)++;
}

list<Pose> NavigationUtilities::cleanHistory(vector<string> *history, EvaluationRecords *record_history) {
  vector<string>::iterator it_history = history->begin();
  list<Pose> tmp_history;
  for (it_history; it_history != prev(history->end(), 1); it_history++) {
    if ((*it_history).back() == '1') {
      tmp_history.push_back(record_history->getPoseFromEncoding(*it_history));
    }
  }
  return tmp_history;
}

void NavigationUtilities::printResult(long newSensedCells, long totalFreeCells, double precision,
                 long numConfiguration, double travelledDistance,
                 int numOfTurning, double totalAngle, double totalScanTime, double resolution) {
  cout << "-----------------------------------------------------------------"
       << endl;
  cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells << "[ "<< 100 * float(newSensedCells)/float(totalFreeCells) << " %]" << endl;
  cout << "Total cell visited :" << numConfiguration << endl;
  cout << "Total travelled distance (meters): " << travelledDistance << endl;
//  cout << "Total travel time: " << travelledDistance / resolution << "s, "
//       << (travelledDistance / resolution) / 60 << " m" << endl;
//  cout << "I came back to the original position since i don't have any other "
//          "candidate position"
//       << endl;
//  cout << "Total exploration time (s): " << travelledDistance / resolution << endl;
//  cout << "Total number of turning: " << numOfTurning << endl;
  cout << "Sum of scan angles (radians): " << totalAngle << endl;
  cout << "Total time for scanning: " << totalScanTime << endl;
//  cout << "Total time for exploration: "
//       << travelledDistance / resolution + totalScanTime << "s, "
//       << (travelledDistance / resolution + totalScanTime) / 60 << " m" << endl;
  if (newSensedCells < precision * totalFreeCells) {
    cout << "FINAL: MAP NOT EXPLORED! :(" << endl;
  } else {
    cout << "FINAL: MAP EXPLORED!" << endl;
  }

  cout << "-----------------------------------------------------------------"
       << endl;
}

Pose NavigationUtilities::getCurrentPose(float resolution, float costresolution, dummy::Map *map,
                    double initFov, int initRange) {
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
    cout << "TRANSFORMS ARE COCKED-UP PAL! Why is that :=> " << ex.what() << endl;
  }

  tf::Vector3 start_position = start_pose_in_tf.getOrigin();
//  cout << "Origin: " << start_position.x() << ", " << start_position.y() << endl;
  tf::Quaternion start_orientation = start_pose_in_tf.getRotation();

  geometry_msgs::PoseStamped start_pose;
  start_pose.header.stamp = start_pose_in_tf.stamp_;
  start_pose.header.frame_id = start_pose_in_tf.frame_id_;

  tf::pointTFToMsg(start_position, start_pose.pose.position);
  tf::quaternionTFToMsg(start_orientation, start_pose.pose.orientation);

  float initX = roundf((start_pose.pose.position.x ) * 100) / 100;
  float initY = roundf((start_pose.pose.position.y ) * 100) / 100;
  tf::Quaternion quat = tf::Quaternion(
      start_pose.pose.orientation.x, start_pose.pose.orientation.y,
      start_pose.pose.orientation.z, start_pose.pose.orientation.w);
  tfScalar angle = roundf(2 * atan2(quat[2], quat[3]) * 100) / 100;

  cout << "Current position in the " << map_frame << " frame:" << initX << "," << initY
       << " with orientation :" << angle << "(" << (angle * 180 / M_PI)
       << " deg)" << endl;

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
  ///pose.getX() * costresolution<< "," << pose.getY() * costresolution << endl;
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



bool NavigationUtilities::move(float x, float y, float orientation, float time_travel,
          list<Pose> *tabuList,
          std::list<std::pair<float, float> > *posToEsclude) {
  move_base_msgs::MoveBaseGoal goal;
  bool success = false;


  MoveBaseClient ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(5.0))) {
    cout << "[pure_navigation@createROSComms]... waiting ..." << endl;
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

  cout << "[pure_navigation.cpp@move] Sending goal = (" << x << "," << y <<
       ") with orientation: " << _orientation << "(" << _orientation * 180 / M_PI
       << ")"<< endl;
  ac.sendGoal(goal);
  //  cout << "[pure_navigation.cpp@move] I'm moving..." << endl;

  actionlib::SimpleClientGoalState curr_state = actionlib::SimpleClientGoalState::PENDING;
  float total_wait_time = 0.0;
  while (!curr_state.isDone()){
    time_travel = std::min(time_travel, (float) 5.0);
    total_wait_time +=  time_travel;
    cout << "     [pure_navigation.cpp@move] Waiting for " << time_travel << " seconds to reach goal" << endl;
    ac.waitForResult(ros::Duration(time_travel));
    curr_state = ac.getState();
    cout << "Result: " << curr_state.getText() << endl;


    if (curr_state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      cout << "[pure_navigation.cpp@move] Goal position reached!" << endl;
      success = true;
    } else if (curr_state.isDone()) {
      cout << "[pure_navigation.cpp@move] The base failed to move, adding this "
              "pose to the Tabulist and posToEsclude" << endl;
      success = false;
      std::pair<int, int> pairToRemove;
      pairToRemove = make_pair(int(x), int(y));
      posToEsclude->push_back(pairToRemove);
    } else if (total_wait_time> 60.0) {
      cout << "[pure_navigation.cpp@move] Is taking too long" << endl;
      success = false;
      curr_state = actionlib::SimpleClientGoalState::ABORTED;
    }
  }
  return success;
}



double NavigationUtilities::getPathLen(std::vector<geometry_msgs::PoseStamped> poses, double robot_radius) {
  double len = 0;
  geometry_msgs::Point p1, p2;
  int npoints = poses.size();
//  cout << "[pure_navigation.cpp@getPathLen]Path has [" <<npoints << "] points" << endl;
  if (npoints > 0) {
    for (int i = 1; i < npoints; i++) {
      p1 = poses[i].pose.position;
      p2 = poses[i - 1].pose.position;
      len += sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
      len += abs(getAngleBetPoses(poses[i], poses[i-1])) * robot_radius;
    }
  } else {
    len = std::numeric_limits<double>::max();
//    cout << "[pure_navigation.cpp@getPathLen]Empty path. Len set to infinite... " << endl;
  }

  return len;
}
double NavigationUtilities::getAngleBetPoses(geometry_msgs::PoseStamped ps1, geometry_msgs::PoseStamped ps2){

  // ....................
  // find quaternion to go from q1 to q2
  // q2 = qr *q1
  // qr = q2 * q1_inv

  double roll, pitch, yaw;
  // inverted quaternion is just inverting w component
  tf::Quaternion q1_inv(ps1.pose.orientation.x,ps1.pose.orientation.y,ps1.pose.orientation.z, -ps1.pose.orientation.w);
  tf::Quaternion q2(ps2.pose.orientation.x,ps2.pose.orientation.y,ps2.pose.orientation.z,ps2.pose.orientation.w);

  tf::Quaternion qr = q2 * q1_inv;
  qr.normalize();

  // and get angles
  tf::Matrix3x3 m(qr);
  m.getRPY(roll, pitch, yaw);
  //
  return yaw;

}

bool NavigationUtilities::showMarkerandNavigate(Pose target, ros::Publisher *marker_pub,
                           nav_msgs::GetPlan *path,
                           ros::ServiceClient *path_client,
                           list<Pose> *tabuList,
                           std::list<std::pair<float, float> > *posToEsclude,
                           double min_robot_speed, double robot_radius) {
  //---------------------------PRINT GOAL POSITION
  geometry_msgs::PointStamped p;
  p.header.frame_id = "map";
  p.header.stamp = ros::Time::now();
  p.point.x = target.getX();
  p.point.y = target.getY();

  marker_pub->publish(p);
  //----------------------------------------------
  move_base_msgs::MoveBaseGoal goal;

  // Update the destination point with the target
  path->request.goal.header.frame_id = "map";
  path->request.goal.pose.position.x = target.getX();
  path->request.goal.pose.position.y = target.getY();
  path->request.goal.pose.orientation.w = 1;

  bool path_srv_call = path_client->call(*path);
  float path_len;
  if (path_srv_call) {
    // calculate path length
    path_len = getPathLen(path->response.plan.poses, robot_radius);
    if (path_len < 1e3) {
      //            printf("[pure_navigation.cpp@showMarkerandNavigate] Path
      //            len is [%3.3f m.]", path_len);
    } else {
      //            printf("[pure_navigation.cpp@showMarkerandNavigate] Path
      //            len is infinite");
      path_len = 1000;
    }
  } else {
    cout << "[pure_navigation.cpp@showMarkerandNavigate] Service call failed! " << endl;
    path_len = 1000;
  }

  float time_travel = 2 * path_len / min_robot_speed;
//      cout << "[pure_navigation.cpp@showMarkerandNavigate] Target is at " <<
//      path_len << " m from the robot" << endl;

  return move(p.point.x, p.point.y, roundf(target.getOrientation() * 100) / 100,
              time_travel, tabuList,
              posToEsclude); // full resolution
}


bool NavigationUtilities::freeInLocalCostmap(Pose target, std::string move_base_local_costmap_topic_name){
  string map_frame= "map";
  bool ans =false;
  int val=0;
  grid_map::GridMap local_grid;
  geometry_msgs::PointStamped targetPoint_map;
  geometry_msgs::PointStamped targetPoint_local;

  targetPoint_map.point.x = target.getX();
  targetPoint_map.point.y = target.getY();
  targetPoint_map.header.frame_id = map_frame;



  // read a local costmap
//      printf("............................................................ \n");

  nav_msgs::OccupancyGridConstPtr local_costmap = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(move_base_local_costmap_topic_name, ros::Duration(1.0));
  if (!local_costmap) {
    printf("oops \n");
  }
//      printf("Local costmap frame is: %s \n",local_costmap->header.frame_id.c_str());

  // cast stupid point into local gridmap frame
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf2_listener(tfBuffer);
  geometry_msgs::TransformStamped map_to_local_tf;

  ros::Time _now_stamp_ = ros::Time(0);

  try {
    map_to_local_tf = tfBuffer.lookupTransform(local_costmap->header.frame_id,map_frame, _now_stamp_,
                                               ros::Duration(5.0));
  } catch (tf2::TransformException &ex) {
    cout << "TRANSFORMS ARE COCKED-UP PAL! Why is that :=> " << ex.what() << endl;
  }
  tf2::doTransform(targetPoint_map, targetPoint_local, map_to_local_tf);

//      printf("In [%s] frame,  point is (%3.1f, %3.1f) \n",targetPoint_map.header.frame_id.c_str(), targetPoint_map.point.x, targetPoint_map.point.y);
//      printf("In [%s] frame,  point is (%3.1f, %3.1f) \n",targetPoint_local.header.frame_id.c_str(), targetPoint_local.point.x, targetPoint_local.point.y);

  // cast occupancy grid to gridmap
  GridMapRosConverter::fromOccupancyGrid(*local_costmap, "layer", local_grid);

  grid_map::Position point(targetPoint_local.point.x, targetPoint_local.point.y);
  // check target cell value inside  local costmap
  if (local_grid.isInside(point)) {
//        printf("Point is INSIDE \n");
    // profit
    val = local_grid.atPosition("layer", point);
//        printf("Value was %d \n",val);
  } else{
//        printf("Point is OUTSIDE ... \n");
  }

  ans = (val == 0);
//      printf("............................................................ \n");
  return ans;

}


Pose NavigationUtilities::selectFreePoseInLocalCostmap(Pose target, list<Pose> *nearCandidates, dummy::Map *map, MCDMFunction *function,
                                  double threshold, ros::ServiceClient *path_client, std::list<std::pair<float, float> > *posToEsclude, EvaluationRecords *record,
                                                       std::string move_base_local_costmap_topic_name)
{
  bool isFreeFromObstacle = false;
  while (isFreeFromObstacle == false) {
    cout << "===> Checking against localmap!" << endl;
    // check that the current target is free from obstacles in the local map
    isFreeFromObstacle = freeInLocalCostmap(target, move_base_local_costmap_topic_name);
    cout << "   isFree: " << isFreeFromObstacle << endl;
    if (isFreeFromObstacle == true) break;// if it's free, the while will be break immediately
    // Otherwise you have to select a new target and continue in the while
    // 1) Add this cell to the list of cell not reachebale
    std::pair<int, int> pairToRemove = make_pair(target.getX(), target.getY());
    posToEsclude->push_back(pairToRemove);
    // remove the current target from the list of candidate position
    cleanPossibleDestination2(nearCandidates, target);
    // Get the list of new candidate position with  associated evaluation
    record = function->evaluateFrontiers(*nearCandidates, map, threshold, path_client);
    // Get a new target
    std::pair<Pose, double> result = function->selectNewPose(record);
    cout << "     record size: " << record->size() << endl;
    target = result.first;
    cout << "     New target selected: " << target.getX() << ", " << target.getY() << endl;
    // and start the new iteration
  }
  return target;
}


