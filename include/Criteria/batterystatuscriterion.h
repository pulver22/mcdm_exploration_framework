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

#ifndef BATTERYSTATUSCRITERION_H
#define BATTERYSTATUSCRITERION_H

#include "criterion.h"
#include "map.h"
#include "pose.h"
// using namespace import_map;
class BatteryStatusCriterion : public Criterion {
public:
  BatteryStatusCriterion(double weight);
  ~BatteryStatusCriterion();
  double evaluate(string currentRobotWayPoint, 
      Pose &p, dummy::Map map, ros::ServiceClient path_client,
      vector<unordered_map<float,
                           std::pair<string, bayesian_topological_localisation::
                                                 DistributionStamped>>>
          mapping_time_belief,
      double batteryTime, GridMap belief_map,
      unordered_map<string, string> mappingWaypoints, prediction_tools tools,
      std::unordered_map<string, double> distances_map);
  double getPathLen(std::vector<geometry_msgs::PoseStamped> poses);
  // only for testing purpose
  // void insertEvaluation(Pose &p, double value);
protected:
  double distance = 0.0;
  double numOfTurning = 0.0;
  double translTime = 0.0;
  double rotTime = 0.0;
  double timeRequired = 0.0;
  double remainingBattery = 0.0;
};

#endif // BATTERYSTATUSCRITERION_H
