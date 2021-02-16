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

double BatteryStatusCriterion::evaluate(string currentRobotWayPoint,
    Pose &p, dummy::Map map, ros::ServiceClient path_client,
    vector<unordered_map<float,
                         std::pair<string, bayesian_topological_localisation::
                                               DistributionStamped>>>
        mapping_time_belief,
    double batteryTime, GridMap belief_map,
    unordered_map<string, string> mappingWaypoints, prediction_tools tools,
    std::unordered_map<string, double> distances_map) {

  // double path_len = Criterion::computeMetricDistance(p, map, path_client);
  double start = ros::Time::now().toSec();
  //   double path_len =
  //       Criterion::computeTopologicalDistance(p, path_client, mappingWaypoints);
  double path_len = Criterion::getPathLenFromMatrix(currentRobotWayPoint, p, distances_map, mappingWaypoints);
  translTime = path_len / TRANSL_SPEED;
  remainingBattery = batteryTime - translTime;
  remainingBattery = max(remainingBattery, 0.0);
  // cout << "BSCriterion: " << ros::Time::now().toSec() - start << endl;
  Criterion::insertEvaluation(p, remainingBattery);
  return remainingBattery;
}
