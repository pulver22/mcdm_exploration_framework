//
// Created by pulver on 29/07/2019.
//
#ifndef RADVARIANCECRITERION_H
#define RADVARIANCECRITERION_H

#include "criterion.h"
#include "map.h"
#include "pose.h"
#include <vector>
#include "utils.h"

class RadVarianceCriterion : public Criterion {
public:
  RadVarianceCriterion(double weight);
  ~RadVarianceCriterion();
  double evaluate(string currentRobotWayPoint,
      Pose &p, dummy::Map map, ros::ServiceClient path_client,
      vector<unordered_map<float,
                           std::pair<string, bayesian_topological_localisation::
                                                 DistributionStamped>>>
          mapping_time_belief,
      double batteryTime, GridMap belief_map,
      unordered_map<string, string> mappingWaypoints, prediction_tools tools,
      std::unordered_map<string, double> distances_map);
      
  // double getTopoNodeValue(Pose p, unordered_map<string, string> mappingWaypoints,
  //     vector<bayesian_topological_localisation::DistributionStamped>
  //       belief_topomaps);

// private:
//   void normalize(long minSensedX, int number);
//   int *intersect(int p1x, int p1y, int p2x, int p2y, Pose &p);


protected:
  double RadVarianceInfoGain = 0.0;
  double tmp_belief = 0.0;
  float _free_space_val = 1.0; // or 1.0
  Utilities _utils;
};

#endif // RADVARIANCECRITERION_H
