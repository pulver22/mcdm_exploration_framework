/*
 * Copyright 2015 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "Criteria/criterion.h"
#include "evaluationrecords.h"

Criterion::Criterion() {}

Criterion::Criterion(string name, double weight, bool highGood)
    : name(name), weight(weight), maxValue(std::numeric_limits<double>::min()),
      minValue(std::numeric_limits<double>::max()), highGood(highGood) {}

Criterion::~Criterion() {}

void Criterion::insertEvaluation(Pose &p, double value) {
  //    if(evaluation.contains(point))
  //        lprint << "#repeated frontier!!!" << endl;

  // string pose = getEncodedKey(p);
  EvaluationRecords *record = new EvaluationRecords();
  string pose = record->getEncodedKey(p);
  evaluation.emplace(pose, value);
  if (isnan(value)) {
    value = 0;
  }
  if (value >= maxValue)
    maxValue = value;
  if (value <= minValue)
    minValue = value;

  delete record;
  // pose.clear();
}

void Criterion::clean() {
  //    for(QHash<SLAM::Geometry::Frontier *, double>::iterator it =
  //    evaluation.begin(); it!=evaluation.end(); it++){
  //        delete it.key();
  //    }
  evaluation.clear();
}

void Criterion::normalize() {
  if (highGood)
    normalizeHighGood();
  else
    normalizeLowGood();
}

void Criterion::normalizeHighGood() {
  unordered_map<string, double> temp;
  for (unordered_map<string, double>::iterator it = evaluation.begin();
       it != evaluation.end(); it++) {
    pair<string, double> p = *it;
    double value = p.second;
    value = (value - minValue) / (maxValue - minValue);
    temp.emplace(p.first, value);
  }
  evaluation = temp;
}

void Criterion::normalizeLowGood() {
  unordered_map<string, double> temp;
  for (unordered_map<string, double>::iterator it = evaluation.begin();
       it != evaluation.end(); it++) {
    pair<string, double> p = *it;
    double value = p.second;
    value = (maxValue - value) / (maxValue - minValue);
    temp.emplace(p.first, value);
  }
  evaluation = temp;
}

double Criterion::getEvaluation(Pose &p) const {

  // string pose = getEncodedKey(p);
  EvaluationRecords *record = new EvaluationRecords();
  string pose = record->getEncodedKey(p);
  double value = evaluation.at(pose);
  delete record;
  if (isnan(value)) {
    value = 0.0;
    // cout << "Name: " << this->name << ", Value: " << value << endl;
  }
  return value;
}

string Criterion::getName() { return name; }

double Criterion::getWeight() { return weight; }

void Criterion::setName(string name) { this->name = name; }

void Criterion::setWeight(double weight) { this->weight = weight; }

string Criterion::getEncodedKey(Pose &p) {

  string key = to_string(p.getX()) + "/" + to_string(p.getY()) + "/" +
               to_string(p.getOrientation()) + "/" + to_string(p.getRange()) +
               "/" + to_string(p.getFOV());

  return key;
}

double Criterion::computeTopologicalDistance(
    Pose &p, ros::ServiceClient *path_client,
    unordered_map<string, string> *mappingWaypoints) {

  double path_len = 0;

  // Topological map
  strands_navigation_msgs::GetRouteTo path;
  string waypointName;
  EvaluationRecords record;
  bool found = false;
  string encoding = record.getEncodedKey(p);
  auto search = mappingWaypoints->find(encoding);
  if (search != mappingWaypoints->end()) {
    waypointName = search->second;
    found = true;
  } else {
    found = false;
  }
  if (found == true) {
    path.request.goal = waypointName;
    bool path_srv_call = path_client->call(path);
    if (path_srv_call) {
      path_len = path.response.route.source.size();
    }
  } else {
    path_len = 1000;
  }

  // path_len = number of edges between current and destination pose
  // 3 is assumed to be the distance in meters between two adjacent nodes
  path_len = 3*path_len;
  return path_len;
}

double Criterion::computeMetricDistance(Pose &p, dummy::Map *map, ros::ServiceClient *path_client) {
  // cout << "travel " << endl;
  Astar astar;
  Pose robotPosition = map->getRobotPosition();
  double path_len = 0;

  // Metric map
  // Update starting point in the path
  nav_msgs::GetPlan path;
  path.request.start.header.frame_id = "map";
  path.request.start.pose.position.x = robotPosition.getX();
  path.request.start.pose.position.y = robotPosition.getY();
  path.request.start.pose.orientation.w = 1;
  path.request.goal.header.frame_id = "map";
  path.request.goal.pose.position.x = p.getX();
  path.request.goal.pose.position.y = p.getY();
  path.request.goal.pose.orientation.w = 1;
  bool path_srv_call = path_client->call(path);
  if (path_srv_call) {
    // calculate path length
    path_len = getPathLen(path.response.plan.poses);
    if (isnan(path_len) or path_len < 0.001) {
      path_len = 0;
    } else if (path_len < 1e3) {
      //      ROS_INFO("Path len is [%3.3f m.]",path_len);
    } else {
      //      ROS_INFO("Path len is infinite");
      path_len = 1000;
    }
  } else {
    ROS_INFO("Path_finding Service call failed! ");
    path_len = 1000;
  }
  bool collision =
      map->checkWallsPathPlanningGrid(p.getX(), p.getY(), p.getRange());
  if (collision == true) {
    path_len = 50000;
  }

  return path_len;
}

double Criterion::getPathLen(std::vector<geometry_msgs::PoseStamped> poses) {
  double len = 0;
  geometry_msgs::Point p1, p2;
  int npoints = poses.size();
  //  ROS_INFO("Path has [%d] points",npoints);
  if (npoints > 0) {
    for (int i = 1; i < npoints; i++) {
      p1 = poses[i].pose.position;
      p2 = poses[i - 1].pose.position;
      len += sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }
  } else {
    len = std::numeric_limits<double>::max();
    //    ROS_INFO("Empty path. Len set to infinite... ");
  }

  return len;
}

double Criterion::getPathLenFromMatrix(string currentRobotWayPoint, 
    Pose &p, std::unordered_map<string, double> *distances_map, 
    unordered_map<string, string> *mappingWaypoints){
  string encoding = Criterion::record_.getEncodedKey(p);
  auto search = mappingWaypoints->find(encoding);
  string goal_wp;
  bool found = false;
  if (search != mappingWaypoints->end()) {
    goal_wp = search->second;
    found = true;
  } else {
    found = false;
  }
  string key = currentRobotWayPoint + goal_wp;
  //TODO: convert current waypoint and destination one into index and then retrieve distance
  return distances_map->at(key);
}