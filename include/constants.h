#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>
// #include "RadarModel.hpp"

//Battery constants
#define MAX_BATTERY 10000.0
#define BATTERY_ONE_PERCENT MAX_BATTERY/100

//Velocity constants
#define TRANSL_SPEED 0.1
#define ROT_SPEED 0.1

const int MAX_TABULIST_COUNT = 0;

struct prediction_tools {
  vector<ros::ServiceClient> pf_stateless_update_srv_list;
  vector<ros::ServiceClient> radarmodel_fake_reading_srv_list;
  vector<bayesian_topological_localisation::DistributionStamped> prior_distributions;
  list<Pose> topoMap;
};


#endif