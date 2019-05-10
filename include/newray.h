#ifndef NEWRAY_H
#define NEWRAY_H

#include <vector>
#include "math.h"
#include "map.h"
#include <utility>

using namespace dummy;

class NewRay
{

public:

  NewRay();
  //~Ray();
  void findCandidatePositions(dummy::Map &map, long posX, long posY, int orientation, double FOV, int range);
  void findCandidatePositions2(dummy::Map &map, long posX, long posY, int orientation, double FOV, int range);
  int isCandidate(const dummy::Map &map, long i, long j);
  int isCandidate2(const dummy::Map &map, long i, long j);
  std::vector<std::pair<long, long> > getCandidatePositions();
  pair< double, double > getSensingTime(const dummy::Map& map, long int posX, long int posY, int orientation, double FOV, int range);
  int performSensingOperation(dummy::Map *map, long posX, long posY, int orientation, double FOV, int range, double firstAngle, double lastAngle);
  void emptyCandidatePositions();
  int getInformationGain(const dummy::Map &map, long posX, long posY, int orientation, double FOV, int range);
  long convertPoint(long y);
  long convertPointPP(long y);
  void calculateInfoGainSensingTime (const dummy::Map &map, long posX, long posY, int orientation, double FOV, int range);
  void setGridToPathGridScale(float value);
protected:
  double mapX, mapY;			//coordinates in the map
  long posX, posY;		//starting position of the robot
  int orientation;			//orientation of the robot (0, 90, 180, 270 degrees)
  double FOV;
  int range;			//range of the scanner
  std::vector<std::pair<long, long> > edgePoints;
  long numGridCols;
  long numGridRows;
  long numPathPlanningGridRows;
  long numPathPlanningGridCols;
  long informationGain;
  double sensingTime;
  float gridToPathGridScale;
};


#endif
