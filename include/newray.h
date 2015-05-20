#ifndef NEWRAY_H
#define NEWRAY_H

#include <vector>
#include "math.h"
#include "map.h"

using namespace dummy;

class NewRay
{
  
public:
  
  NewRay();
  //~Ray();
  void findCandidatePositions(dummy::Map &map, long posX, long posY, int orientation, double FOV, int range);
  int isCandidate(const dummy::Map &map, long i, long j);
  std::vector<std::pair<long, long> > getCandidatePositions();
  double getSensingTime(const dummy::Map &map, long posX, long posY, int orientation, double FOV, int range);
  void performSensingOperation(dummy::Map &map, long posX, long posY, int orientation, double FOV, int range);
  void emptyCandidatePositions();
  int getInformationGain(const dummy::Map &map, long posX, long posY, int orientation, double FOV, int range);
  long convertPoint(long y);
  void calculateInfoGainSensingTime (const dummy::Map &map, long posX, long posY, int orientation, double FOV, int range);
  
protected:
  double mapX, mapY;			//coordinates in the map
  long posX, posY;		//starting position of the robot
  int orientation;			//orientation of the robot (0, 90, 180, 270 degrees)
  double FOV;
  int range;			//range of the scanner
  std::vector<std::pair<long, long> > edgePoints;
  long numGridCols;
  long numGridRows;
  long informationGain;
  double sensingTime;
};


#endif