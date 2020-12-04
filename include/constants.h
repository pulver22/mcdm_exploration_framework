#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>
// #include "RadarModel.hpp"

//Battery constants
#define MAX_BATTERY 10000.0
#define BATTERY_ONE_PERCENT MAX_BATTERY/100

//Velocity constants
#define TRANSL_SPEED 0.5
#define ROT_SPEED 0.1

const int MAX_TABULIST_COUNT = 0;

struct RFID_tools {
  // RadarModel *rm;
  std::vector<std::pair<double,double>> tags_coord;
  double freq;
  double txtPower;
  double sensitivity;
  std::vector<RFIDGridmap>* RFID_maps_list;
};


#endif