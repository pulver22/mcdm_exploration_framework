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

#ifndef CRITERION_H
#define CRITERION_H
#include "map.h"
#include "pose.h"
#include <string>
#include <unordered_map>
// #include "RadarModel.hpp"
#include "PathFinding/astar.h"
#include "bayesian_topological_localisation/DistributionStamped.h"
#include "constants.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/GetPlan.h"
#include "strands_navigation_msgs/GetRouteTo.h"
#include <boost/numeric/ublas/symmetric.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "evaluationrecords.h"

namespace bnu = boost::numeric::ublas;

// using namespace import_map;
using namespace std;
using namespace dummy;
class Criterion {
public:
  // Constructor and destructor
  Criterion();
  Criterion(string name, double weight, bool highGood);
  ~Criterion();

  // Other methods
  virtual double evaluate(string currentRobotWayPoint, 
      Pose &p, dummy::Map *map, ros::ServiceClient *path_client,
      vector<unordered_map<float,
                           std::pair<string, bayesian_topological_localisation::
                                                 DistributionStamped>>>
          *mapping_time_belief,
      double *batteryTime, GridMap *belief_map,
      unordered_map<string, string> *mappingWaypoints,
      prediction_tools *tools,
      std::unordered_map<string, double> *distances_map){};
  double getEvaluation(Pose &p) const;
  void insertEvaluation(Pose &p, double value);
  void clean();
  void normalize();
  double computeMetricDistance(Pose &p, dummy::Map *map, ros::ServiceClient *path_client);
  double computeTopologicalDistance(Pose &p, ros::ServiceClient *path_client,
      unordered_map<string, string> *mappingWaypoints);
  double getPathLen(std::vector<geometry_msgs::PoseStamped> poses);

  double getPathLenFromMatrix(string currentRobotWayPoint, 
    Pose &p, std::unordered_map<string, double> *distances_map,
    unordered_map<string, string> *mappingWaypoints);

  // Setters and getters
  string getName();
  double getWeight();
  void setName(string name);
  void setWeight(double weight);

private:
  void normalizeHighGood();
  void normalizeLowGood();
  string getEncodedKey(Pose &p);

protected:
  string name;
  double weight = 0.0;
  bool highGood;
  double maxValue, minValue;
  EvaluationRecords record_;

private:
  unordered_map<string, double> evaluation_;
};

#endif // CRITERION_H
