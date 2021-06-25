//
// Created by pulver on 29/07/2019.
//

#include "Criteria/radvariancecriterion.h"
#include "Criteria/criteriaName.h"
#include "Eigen/Eigen"
#include "newray.h"
#include "utils.h"
#include <algorithm> // std::find
#include <math.h>

#include "bayesian_topological_localisation/DistributionStamped.h"
// #include "bayesian_topological_localisation/Predict.h"
// #include "bayesian_topological_localisation/UpdatePriorLikelihoodObservation.h"
#include "rfid_grid_map/GetFakeBeliefMaps.h"

using namespace dummy;
using namespace grid_map;

RadVarianceCriterion::RadVarianceCriterion(double weight)
    : Criterion(RAD_VARIANCE, weight, true) { // true maximises
  // minValue = 0.0;
}

RadVarianceCriterion::~RadVarianceCriterion() {}

double RadVarianceCriterion::evaluate(string currentRobotWayPoint,
    Pose &p, dummy::Map map, ros::ServiceClient path_client,
    vector<unordered_map<float,
                         std::pair<string, bayesian_topological_localisation::
                                               DistributionStamped>>>
        mapping_time_belief,
    double batteryTime, GridMap belief_map,
    unordered_map<string, string> mappingWaypoints, prediction_tools tools,
    std::unordered_map<string, double> distances_map) {

  this->RadVarianceInfoGain = 0;
  
  vector<bayesian_topological_localisation::DistributionStamped>
      values = tools.var_values_distribution;

  // 1) Compute entropy on a single waypoint
  this->RadVarianceInfoGain = getTopoNodeValue(p, mappingWaypoints, values);
  // ORI) Read values from GP-prediction
  // pair<float,float> current_pose = make_pair(p.getX(), p.getY());
  // int index = 0;
  
  // auto it = find(tools.coordinates.begin(), tools.coordinates.end(), current_pose);
  // if (it != tools.coordinates.end() )
  // {
  //   index = it - tools.coordinates.begin();
  //   this->RadVarianceInfoGain = tools.var_values[index];
  // }else{
  //   this->RadVarianceInfoGain = 0;
  // }
  
  Criterion::insertEvaluation(p, this->RadVarianceInfoGain);
  return this->RadVarianceInfoGain;
}

