/* Copyright 2015 <copyright holder> <email>
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

#include "Criteria/batterystatuscriterion.h"
#include "Criteria/criteriaName.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/GetPlan.h"
#include <iostream>

BatteryStatusCriterion::BatteryStatusCriterion(double weight)
    : Criterion(BATTERY_STATUS, weight, true) {}

BatteryStatusCriterion::~BatteryStatusCriterion() {}

double BatteryStatusCriterion::evaluate(
    Pose &p, dummy::Map *map, ros::ServiceClient *path_client, vector<unordered_map<float,  std::pair<string, bayesian_topological_localisation::DistributionStamped>>> *mapping_time_belief,
    double *batteryTime, GridMap *belief_map,
    unordered_map<string, string> *mappingWaypoints,
    prediction_tools *tools) {
  // Pose robotPosition = map->getRobotPosition();
  // nav_msgs::GetPlan path;
  // double path_len = 0;
  // double startX_meter, startY_meter;
  // double goalX_meter, goalY_meter;

  // // Update starting point in the path
  // path.request.start.header.frame_id = "map";
  // path.request.start.pose.position.x = robotPosition.getX();
  // path.request.start.pose.position.y = robotPosition.getY();
  // path.request.start.pose.orientation.w = 1;

  // path.request.goal.header.frame_id = "map";
  // path.request.goal.pose.position.x = p.getX();
  // path.request.goal.pose.position.y = p.getY();
  // path.request.goal.pose.orientation.w = 1;

  // bool path_srv_call = path_client->call(path);
  // if (path_srv_call) {
  //   // calculate path length
  //   path_len = getPathLen(path.response.plan.poses);
  //   if (isnan(path_len) or path_len < 0.001) {
  //     path_len = 0;
  //   } else if (path_len < 1e3) {
  //     //      ROS_INFO("Path len is [%3.3f m.]",path_len);
  //   } else {
  //     //      ROS_INFO("Path len is infinite");
  //     path_len = 1000;
  //   }
  // } else {
  //   ROS_INFO("Path_finding Service call failed! ");
  //   path_len = 1000;
  // }
  // bool collision =
  //     map->checkWallsPathPlanningGrid(p.getX(), p.getY(), p.getRange());
  // if (collision == true) {
  //   path_len = 50000;
  // }

  double path_len = Criterion::computeMetricDistance(
        p, map, path_client, batteryTime, belief_map,
      mappingWaypoints, &(tools->prior_distributions));
  translTime = path_len / TRANSL_SPEED;
  remainingBattery = *batteryTime - translTime;
  remainingBattery = max(remainingBattery, 0.0);

  Criterion::insertEvaluation(p, remainingBattery);
  return path_len;
}

double BatteryStatusCriterion::getPathLen(
    std::vector<geometry_msgs::PoseStamped> poses) {
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