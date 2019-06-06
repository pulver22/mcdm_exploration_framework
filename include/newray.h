#ifndef NEWRAY_H
#define NEWRAY_H

#include "map.h"
#include "math.h"
#include <utility>
#include <vector>

using namespace dummy;

class NewRay {

public:
  NewRay();
  //~Ray();
  void findCandidatePositions(dummy::Map *map, double posX_meter,
                              double posY_meter, int orientation, double FOV,
                              int range);
  void findCandidatePositions2(dummy::Map *map, double posX_meter,
                               double posY_meter, int orientation, double FOV,
                               int range);
  int isCandidate(dummy::Map *map, double cell_i, double cell_j);
  int isCandidate2(dummy::Map *map, double cell_i, double cell_j);
  std::vector<std::pair<long, long>> getCandidatePositions();
  pair<double, double> getSensingTime(dummy::Map *map, double posX_meter,
                                      double posY_meter, float orientation,
                                      double FOV, int range);
  int performSensingOperation(dummy::Map *map, double posX_meter,
                              double posY_meter, int orientation, double FOV,
                              int range, double firstAngle, double lastAngle);
  void emptyCandidatePositions();
  int getInformationGain(dummy::Map *map, double posX_meter, double posY_meter,
                         int orientation, double FOV, int range);
  long convertPoint(long y);
  long convertPointPP(long y);
  void setGridToPathGridScale(float value);

protected:
  double mapX, mapY; // coordinates in the map
  long posX, posY;   // starting position of the robot
  int orientation;   // orientation of the robot (0, 90, 180, 270 degrees)
  double FOV;
  int range; // range of the scanner
  std::vector<std::pair<long, long>> edgePoints;
  long numGridCols;
  long numGridRows;
  long numPathPlanningGridRows;
  long numPathPlanningGridCols;
  long informationGain;
  double sensingTime;
  float gridToPathGridScale;
};

#endif
