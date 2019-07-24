#ifndef POSE_H
#define POSE_H
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/Polygon.hpp"
#include <grid_map_core/grid_map_core.hpp>

#include "grid_map_ros/GridMapRosConverter.hpp"
#include <grid_map_ros/grid_map_ros.hpp>

#include "grid_map_cv/grid_map_cv.hpp"
#include <utility>
using namespace grid_map;

class Pose {
public:
  Pose();
  Pose(float aX, float aY, float orientation, int range, double FOV);
  virtual ~Pose();
  double getDistance(Pose &pose);
  float getX();
  float getY();
  float getOrientation();
  int getRange();
  double getFOV();
  bool isEqual(Pose &p);
  int getInformationGain();
  void setInformationGain(int value);
  bool operator==(const Pose &p);
  void setScanAngles(std::pair<double, double> angles);
  void setOrientation(float orientation);
  std::pair<double, double> getScanAngles();
  void updateFromGridMapPosition(grid_map::Position p, float orientation, int range, double FOV);

protected:
  float aX, aY;      // x and y coordinates of the cell
  float orientation; // orientation theta of th robot
  int range;         // radius of the sensing operation
  double FOV;        // central angle of the sensing operation
  int informationGain;
  std::pair<double, double> scanAngles;
};

#endif