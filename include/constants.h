#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>
// #include "RadarModel.hpp"

//Battery constants
#define MAX_BATTERY 100000.0
#define BATTERY_ONE_PERCENT MAX_BATTERY/100

//Velocity constants
#define TRANSL_SPEED 1.0
#define ROT_SPEED 1.0

#define EVALUATION_TIME 70.0

const int MAX_TABULIST_COUNT = 10;  // This should be equal to the number of nodes in the map

struct prediction_tools {
  vector<ros::ServiceClient> pf_stateless_update_srv_list;
  vector<ros::ServiceClient> radarmodel_fake_reading_srv_list;
  vector<bayesian_topological_localisation::DistributionStamped> mean_values_distribution;
  vector<bayesian_topological_localisation::DistributionStamped> var_values_distribution;
  list<Pose> topoMap;
  vector<pair<float, float>> coordinates;
  vector<float> x_values;
  vector<float> y_values;
  vector<float> mean_values;
  vector<float> var_values;
};


#endif