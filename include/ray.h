#ifndef RAY_H
#define RAY_H

#include <vector>
#include "math.h"
#include "map.h"


using namespace dummy;
class Ray
{
  
public:
  
  Ray();
  //~Ray();
  void findCandidatePositions(dummy::Map &map, long posX, long posY, int orientation, double FOV, int range);
  std::vector<std::pair<long,long> > getCandidatePositions();
  void setGrid(const dummy::Map &map);
  int getInformationGain(const dummy::Map &map, long posX, long posY, int orientation, double FOV, int range);
  void performSensingOperation(dummy::Map &map, long posX, long posY, int orientation, double FOV, int range);
  int getGridValue(long i, long j);
  void emptyCandidatePositions();
  
private:
  double mapX, mapY;			//coordinates in the map
  long posX, posY;		//starting position of the robot
  int orientation;			//orientation of the robot (0, 90, 180, 270 degrees)
  double FOV;
  int range;			//range of the scanner
  std::vector<std::pair<long, long> > edgePoints;
  std::vector<long> grid;
  void setGridValue(long i, long j, int value);
  long numGridCols;
  long numGridRows;
};


#endif