#include <vector>
#include "math.h"
#include "map.h"

#include "newray.h"

# define PI           3.14159265358979323846  /* pi */



NewRay::NewRay()
{
}


//Check if a cell is candidate position: return 1 if the cell is adjacent to at least one free cell, 0 otherwise
int NewRay::isCandidate(dummy::Map *map, double cell_i, double cell_j)
{

  int candidate = 0;
  long r = cell_i;
  long s = cell_j;
  long minR = r - 1, maxR = r + 1, minS = s -1, maxS = s + 1;
  if(minR < 0) minR = 0;
  if(minS < 0) minS = 0;
  if(maxR > map->getPathPlanningNumRows()) maxR = map->getPathPlanningNumRows();
  if(maxS > map->getPathPlanningNumCols()) maxS = map->getPathPlanningNumCols();


  for(r = minR; r <= maxR; ++r)
  {
    for(s = minS; s <= maxS; ++s)
    {
      if (map->getPathPlanningGridValue(r, s) == 0) candidate = 1;
    }
  }
  return candidate;

}

int NewRay::isCandidate2(dummy::Map *map, double cell_i, double cell_j)
{

  int candidate = 0;
  long r = cell_i;
  long s = cell_j;
  long minR = r - 1, maxR = r + 1, minS = s -1, maxS = s + 1;
  if(minR < 0) minR = 0;
  if(minS < 0) minS = 0;
  if(maxR > map->getPathPlanningNumRows()) maxR = map->getPathPlanningNumRows();
  if(maxS > map->getPathPlanningNumCols()) maxS = map->getPathPlanningNumCols();


  for(r = minR; r <= maxR; ++r)
  {
    for(s = minS; s <= maxS; ++s)
    {
      for(int rg = r*gridToPathGridScale; rg < r*gridToPathGridScale + gridToPathGridScale; ++rg)
      {
        for(int sg = s*gridToPathGridScale; sg < s*gridToPathGridScale + gridToPathGridScale; ++sg)
        {
          if (map->getGridValue(rg, sg) == 0) candidate = 1;
        }
      }
    }
  }
  return candidate;

}


//finds the candidate positions: cells already scanned in range of the robot which are adjacent to at least one free cell
void NewRay::findCandidatePositions(dummy::Map *map, double posX_meter, double posY_meter, int orientation, double FOV, int range)
{
  NewRay::numGridRows = map->getNumGridRows();
  NewRay::numPathPlanningGridCols = map->getPathPlanningNumCols();
  NewRay::numPathPlanningGridRows = map->getPathPlanningNumRows();
  NewRay::gridToPathGridScale = map->getGridToPathGridScale();

  //set the correct FOV orientation
  double startingPhi = orientation*PI/180 - FOV/2;
  double endingPhi = orientation*PI/180 + FOV/2;
  int add2pi = 0;

  // We are now workin on the path planning grid with coarse resolution
  long cell_i, cell_j;
  map->getPathPlanningIndex(posX_meter, posY_meter, cell_i, cell_j);



  if(startingPhi <= 0)
  {
    add2pi = 1;
    startingPhi = 2*PI + startingPhi;
    endingPhi = 2*PI + endingPhi;

  }
  if(endingPhi > 2*PI) add2pi = 1;

//  std::cout << std::endl << "[newRay.cpp@findCandidatePositions] StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;
//  std::cout << "[newray.cpp@findCandidatePositions] [posX, posY](cells): " << cell_i << ", " << cell_j <<std::endl;
//  std::cout << "[newray.cpp@findCandidatePositions] Range (cells): " << range << std::endl;
//  std::cout << "[newray.cpp@findCandidatePositions] gridToPathGridScale: " << gridToPathGridScale << std::endl;

  //select the portion of map to be scanned
  long minI = cell_i - range;
  long maxI = cell_i + range;
  long minJ = cell_j - range;
  long maxJ = cell_j + range;

  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getPathPlanningNumRows()) maxI = map->getPathPlanningNumRows();
  if(maxJ > map->getPathPlanningNumCols()) maxJ = map->getPathPlanningNumCols();
//  cout << "[newray.cpp@findCandidatePositions] [minI, minJ]: " << minI << "," << minJ << ", [maxI, maxJ]: " << maxI << ", " << maxJ << endl;

  double x_meter, y_meter;

  //scan the cells in the selected portion of the map
  for(long i = minI; i <= maxI; ++i)
  {
    for(long j = minJ; j <=maxJ; ++j)
    {

      double distance = sqrt((i - cell_i)*(i - cell_j) + (j - cell_j)*(j - cell_j));
      //if a cell is a candidate one and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getPathPlanningGridValue(i, j) == 2 && distance <= range)
      {
//        cout <<"[newRay.cpp@findCandidatePositions] [i,j] = [" << i <<","<<j<<"], value: " << map->getPathPlanningGridValue(i, j) << ", distance: " << distance << ", range: " <<range << endl;
        if(NewRay::isCandidate(map, i, j) == 1)
        {

          double curX = cell_i;		//starting position of the ray
          double curY = cell_j;
          double robotX = cell_i;		//position of the robot
          double robotY = cell_j;

          double convertedI = NewRay::convertPointPP(i);
          double convertedRX = NewRay::convertPointPP(robotX);

          double slope = atan2(NewRay::convertPointPP(i) - NewRay::convertPointPP(robotX), j - robotY);	//calculate the slope of the ray with atan2

          if(slope <= 0 && add2pi == 0) slope = slope + 2*PI;
          if(add2pi == 1) slope = 2*PI + slope;		//needed in case of FOV spanning from negative to positive angle values


          if(slope >= startingPhi && slope <= endingPhi)	//only cast the ray if it is inside the FOV of the robot
          {
            //raycounter++;
//            std::cout << "[newRay.cpp@findCandidatePositions] Inside loop, slope: " << slope  << " Cell: " << j << " " << i << std::endl;

            int hit = 0;			//set to 1 when obstacle is hit by ray or when the cell is reached in order to stop the ray
            double u = 0;			//current position along the ray

            while(hit == 0)		//scan the map along the ray until an ostacle is found or the considered cell is reached
            {

              //convert the position on the ray to cell coordinates to check the grid
              curY = robotY + 0.5 + u*cos(slope);
              curX = robotX + 0.5 - u*sin(slope);

              //not needed, but left anyway
              if(curX < 0 || curX > map->getPathPlanningNumRows() || curY < 0 || curY > map->getPathPlanningNumCols()) hit = 1;

              if(map->getPathPlanningGridValue((long)curX, (long)curY) == dummy::Map::CellValue::OBST)
              {
                hit = 1;		//hit set to 1 if an obstacle is found
//                std::cout << "[newRay.cpp@findCandidatePositions]HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
              }


              if((long)curX == i && (long)curY == j)	//if the free cell is reached, save it as edge point and stop the ray.
              {
                std::pair<long,long> temp = std::make_pair(i, j);
                NewRay::edgePoints.push_back(temp);
//                std::cout << "[newRay.cpp@findCandidatePositions]Cell scanned: " << (int)curY << " " << (int)curX << std::endl;
                map->getPathPlanningPosition(x_meter, y_meter, i, j);
//                std::cout << "[newRay.cpp@findCandidatePositions]   Cell scanned: " << x_meter << " " << y_meter << std::endl;
                hit = 1;
              }
              u += 0.2;		//move forward along the ray
            }
          }
        }
      }
    }
  }
}

//finds the candidate positions: cells already scanned in range of the robot which are adjacent to at least one free cell
void NewRay::findCandidatePositions2(dummy::Map *map, double posX_meter, double posY_meter, int orientation, double FOV, int range)
{
  NewRay::numGridRows = map->getNumGridRows();
  NewRay::numPathPlanningGridCols = map->getPathPlanningNumCols();
  NewRay::numPathPlanningGridRows = map->getPathPlanningNumRows();
  NewRay::gridToPathGridScale = map->getGridToPathGridScale();

  //set the correct FOV orientation
  double startingPhi = orientation*PI/180 - FOV/2;
  double endingPhi = orientation*PI/180 + FOV/2;
  int add2pi = 0;

  // We are now workin on the path planning grid with coarse resolution
  long cell_i, cell_j;
  map->getPathPlanningIndex(posX_meter, posY_meter, cell_i, cell_j);

  if(startingPhi <= 0)
  {
    add2pi = 1;
    startingPhi = 2*PI + startingPhi;
    endingPhi = 2*PI + endingPhi;

  }
  if(endingPhi > 2*PI) add2pi = 1;

  //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

  //select the portion of map to be scanned
  long minI = cell_i - range;
  long maxI = cell_i + range;
  long minJ = cell_j - range;
  long maxJ = cell_j + range;

  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getPathPlanningNumRows()) maxI = map->getPathPlanningNumRows();
  if(maxJ > map->getPathPlanningNumCols()) maxJ = map->getPathPlanningNumCols();

  //scan the cells in the selected portion of the map
  for(long i = minI; i <= maxI; ++i)
  {
    for(long j = minJ; j <=maxJ; ++j)
    {

      double distance = sqrt((i - cell_i)*(i - cell_i) + (j - cell_j)*(j - cell_j));
      //cout << map->getGridValue(i, j) << " : " << distance << " : " <<range << endl;

      //if a cell is a candidate one and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getPathPlanningGridValue(i, j) == 2 && distance <= range)
      {

        if(NewRay::isCandidate2(map, i, j) == 1)
        {

          double curX = cell_i;		//starting position of the ray
          double curY = cell_j;
          double robotX = cell_i;		//position of the robot
          double robotY = cell_j;

          double convertedI = NewRay::convertPointPP(i);
          double convertedRX = NewRay::convertPointPP(robotX);

          double slope = atan2(NewRay::convertPointPP(i) - NewRay::convertPointPP(robotX), j - robotY);	//calculate the slope of the ray with atan2

          if(slope <= 0 && add2pi == 0) slope = slope + 2*PI;
          if(add2pi == 1) slope = 2*PI + slope;		//needed in case of FOV spanning from negative to positive angle values

          //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

          if(slope >= startingPhi && slope <= endingPhi)	//only cast the ray if it is inside the FOV of the robot
          {
            //raycounter++;
            //std::cout << "Inside loop, slope: " << slope  << " Cell: " << j << " " << i << std::endl;

            int hit = 0;			//set to 1 when obstacle is hit by ray or when the cell is reached in order to stop the ray
            double u = 0;			//current position along the ray

            while(hit == 0)		//scan the map along the ray until an ostacle is found or the considered cell is reached
            {

              //convert the position on the ray to cell coordinates to check the grid
              curY = robotY + 0.5 + u*cos(slope);
              curX = robotX + 0.5 - u*sin(slope);

              //not needed, but left anyway
              if(curX < 0 || curX > map->getPathPlanningNumRows() || curY < 0 || curY > map->getPathPlanningNumCols()) hit = 1;

              if(map->getPathPlanningGridValue((long)curX, (long)curY) == dummy::Map::CellValue::OBST)
              {
                hit = 1;		//hit set to 1 if an obstacle is found
                //std::cout << "HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
              }


              if((long)curX == i && (long)curY == j)	//if the free cell is reached, save it as edge point and stop the ray.
              {
                std::pair<long,long> temp = std::make_pair(i, j);
                NewRay::edgePoints.push_back(temp);
                //std::cout << "Cell scanned: " << (int)curY << " " << (int)curX << std::endl;
                hit = 1;
              }
              u += 0.2;		//move forward along the ray
            }
          }
        }
      }
    }
  }
}


vector< std::pair<long,long> > NewRay::getCandidatePositions()
{
  return NewRay::edgePoints;
}

void NewRay::emptyCandidatePositions()
{
  NewRay::edgePoints.clear();
}


//calculate the sensing time of a possible scanning operation, returns the minimum FOV required to scan all the free cells from the considered pose
//ATTENTION: the FOV is always centered in the orientation of the robot
//ATTENTION: in order to optimize the computing time, this method should be fused with the information gain one
std::pair<double,double> NewRay::getSensingTime(dummy::Map *map, double posX_meter, double posY_meter, int orientation, double FOV, int range)
{

  NewRay::numGridRows = map->getNumGridRows();
  setGridToPathGridScale(map->getGridToPathGridScale());

  // We are working on the navigation map with fine resolution
  long cell_i, cell_j;
  map->getGridIndex(posX_meter, posY_meter, cell_i, cell_j);
  range = range * gridToPathGridScale;


  double minPhi = 0;	//slope of the first ray required
  double maxPhi = 0;	//slope of the last ray required
  int phiFound = 0;	//set to 1 if at least a cell can be scanned

  //set the correct FOV orientation
  double startingPhi = orientation*PI/180 - FOV/2;
  double endingPhi = orientation*PI/180 + FOV/2;
  int add2pi = 0;

  if(startingPhi <= 0)
  {
    add2pi = 1;
    startingPhi = 2*PI + startingPhi;
    endingPhi = 2*PI + endingPhi;
  }

  if(endingPhi > 2*PI) add2pi = 1;

  //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

  //select the portion of map to be scanned
//  long minI = posX*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
//  long maxI = posX*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;
//  long minJ = posY*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
//  long maxJ = posY*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;

  long minI = cell_i - range;
  long maxI = cell_j + range;
  long minJ = cell_j - range;
  long maxJ = cell_j + range;

  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getNumGridRows()) maxI = map->getNumGridRows();
  if(maxJ > map->getNumGridCols()) maxJ = map->getNumGridCols();

  //scan the cells in the selected portion of the map
  for(long i = minI; i <= maxI; ++i)
  {
    for(long j = minJ; j <=maxJ; ++j)
    {

      double distance = sqrt((i - cell_i)*(i - cell_i) + (j - cell_j)*(j - cell_j));

      //if a cell is free and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getGridValue(i, j) == 0 && distance <= range)
      {
//        double curX = posX*gridToPathGridScale + gridToPathGridScale/2;		//starting position of the ray
//        double curY = posY*gridToPathGridScale + gridToPathGridScale/2;
//        double robotX = posX*gridToPathGridScale + gridToPathGridScale/2;		//position of the robot
//        double robotY = posY*gridToPathGridScale + gridToPathGridScale/2;

        double curX = cell_i;
        double curY = cell_j;
        double robotX = cell_i;
        double robotY = cell_j;

        double convertedI = NewRay::convertPoint(i);
        double convertedRX = NewRay::convertPoint(robotX);

        double slope = atan2(NewRay::convertPoint(i) - NewRay::convertPoint(robotX), j - robotY);	//calculate the slope of the ray with atan2

        if(slope <= 0 && add2pi == 0) slope = slope + 2*PI;
        if(add2pi == 1) slope = 2*PI + slope;		//needed in case of FOV spanning from negative to positive angle values

//        std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

        if(slope >= startingPhi && slope <= endingPhi)	//only cast the ray if it is inside the FOV of the robot
        {
//          std::cout << "Inside loop, slope: " << slope  << " Cell: " << j << " " << i << std::endl;

          int hit = 0;			//set to 1 when obstacle is hit by ray or when the cell is reached in order to stop the ray
          double u = 0;			//current position along the ray
          while(hit == 0)		//scan the map along the ray until an ostacle is found or the considered cell is reached
          {

            //convert the position on the ray to cell coordinates to check the grid
            curY = robotY + 0.5 + u*cos(slope);
            curX = robotX + 0.5 - u*sin(slope);

            //not needed, but left anyway
            if(curX < 0 || curX > map->getNumGridRows() || curY < 0 || curY > map->getNumGridCols()) hit = 1;

            if(map->getGridValue((long)curX, (long)curY) == dummy::Map::CellValue::OBST)
            {
              hit = 1;		//hit set to 1 if an obstacle is found
//              std::cout << "HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
            }

            if((long)curX == i && (long)curY == j)	//free cell reached, check if the slope is a candidate for first or last ray
            {
              if(phiFound == 0)		//enters if it is the first free cell found
              {
                phiFound = 1;
                minPhi = slope;
                maxPhi = slope;
              }
              if(phiFound == 1)
              {
                if(slope < minPhi) minPhi = slope;
                if(slope > maxPhi) maxPhi = slope;
              }

              hit = 1;
            }
            u += 0.2;		//move forward along the ray
          }
        }
      }
    }
  }
  double value;		//FOV to return

  /*
  if(phiFound == 0) return -1;		//return -1 if no free cells can be scanned
  else 					//return the correct FOV (ALWAYS CENTERED ON THE ORIENTATION)
  {
    if(minPhi - startingPhi <= endingPhi - maxPhi) value = (endingPhi - startingPhi - 2*(minPhi - startingPhi));
	else value = (endingPhi - startingPhi - 2*(endingPhi - maxPhi));
  }

  //std::cout << "startingPhi " << startingPhi << " endingPhi " << endingPhi << " minPhi " << minPhi << " maxPhi " << maxPhi << std::endl;

  return value;
  */
  // return sensingTime;
  std::pair<double, double> angles;
  angles.first = minPhi;
  angles.second = maxPhi;
  return angles;
}


//perform the sensing operation by setting the value of the free cell scanned to 2
int NewRay::performSensingOperation(dummy::Map *map, double posX_meter, double posY_meter, int orientation, double FOV, int range, double firstAngle, double lastAngle)
{
  NewRay::numGridRows = map->getNumGridRows();
  setGridToPathGridScale(map->getGridToPathGridScale());

  // We are working on the navigation map with fine resolution
  long cell_i, cell_j;
  map->getGridIndex(posX_meter, posY_meter, cell_i, cell_j);
  range = range * gridToPathGridScale;

  int counter = 0;

  //set the correct FOV orientation
  double startingPhi = firstAngle; //orientation*PI/180 - FOV/2;
  double endingPhi = lastAngle; //orientation*PI/180 + FOV/2;
  int add2pi = 0;

  if(startingPhi <= 0)
  {
    add2pi = 1;
    startingPhi = 2*PI + startingPhi;
    endingPhi = 2*PI + endingPhi;
  }

  if(endingPhi > 2*PI) add2pi = 1;


  std::cout << endl;
//  std::cout << "[newray.cpp@performSensingOperation] StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;
//  std::cout << "[newray.cpp@performSensingOperation] [posX, posY](cells): " << cell_i << ", " << cell_j <<std::endl;
//  std::cout << "[newray.cpp@performSensingOperation] Range (cells): " << range << std::endl;
//  std::cout << "[newray.cpp@performSensingOperation] gridToPathGridScale: " << gridToPathGridScale << std::endl;

  //select the portion of map to be scanned
//  long minI = posX*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
//  long maxI = posX*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;
//  long minJ = posY*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
//  long maxJ = posY*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;

  long minI = cell_i - range;
  long maxI = cell_j + range;
  long minJ = cell_j - range;
  long maxJ = cell_j + range;

  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getNumGridRows()) maxI = map->getNumGridRows();
  if(maxJ > map->getNumGridCols()) maxJ = map->getNumGridCols();

  cout << "[newray.cpp@performSensingOperation] [minI, minJ]: " << minI << "," << minJ << ", [maxI, maxJ]: " << maxI << ", " << maxJ << endl;

  //scan the cells in the selected portion of the map
  int count = 0;
  for(long i = minI; i <= maxI; ++i)
  {
    for(long j = minJ; j <=maxJ; ++j)
    {
      count ++;
      double distance = sqrt((i - cell_i)*(i - cell_i) + (j - cell_j)*(j - cell_j));
//      std::cout << "[newray.cpp@performSensingOperation][" << count << "] Cell: [" << i << "," << j << "], Value "<< map->getGridValue(i, j) << ", Distance: " << distance << std::endl;
      //if a cell is free and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->isGridValueFree( grid_map::Index(i,j)) && distance <= range)

      {

//        double curX = posX*gridToPathGridScale + gridToPathGridScale/2;		//starting position of the ray
//        double curY = posY*gridToPathGridScale + gridToPathGridScale/2;
//        double robotX = posX*gridToPathGridScale + gridToPathGridScale/2;		//position of the robot
//        double robotY = posY*gridToPathGridScale + gridToPathGridScale/2;
        double curX = cell_i;
        double curY = cell_j;
        double robotX = cell_i;
        double robotY = cell_j;
//        std::cout << "[newray.cpp@performSensingOperation] [robotX, robotY]: " << robotX << ", " << robotY <<std::endl;

        double convertedI = NewRay::convertPoint(i);
        double convertedRX = NewRay::convertPoint(robotX);

        double slope = atan2(NewRay::convertPoint(i) - NewRay::convertPoint(robotX), j - robotY);	//calculate the slope of the ray with atan2

        if(slope <= 0 && add2pi == 0) slope = slope + 2*PI;
        if(add2pi == 1) slope = 2*PI + slope;		//needed in case of FOV spanning from negative to positive angle values

//        std::cout << "[newray.cpp@performSensingOperation] The cell [ " << i << "," << j << " is free and within range of the robot" <<std::endl;

        if(slope >= startingPhi && slope <= endingPhi)	//only cast the ray if it is inside the FOV of the robot
        {
          //raycounter++;
//          std::cout << "[newray.cpp@performSensingOperation] Inside loop, slope: " << slope  << " Cell: " << j << " " << i << std::endl;

          int hit = 0;			//set to 1 when obstacle is hit by ray or when the cell is reached in order to stop the ray
          double u = 0;			//current position along the ray
          while(hit == 0)		//scan the map along the ray until an ostacle is found or the considered cell is reached
          {

            //convert the position on the ray to cell coordinates to check the grid
            curY = robotY + 0.5 + u*cos(slope);
            curX = robotX + 0.5 - u*sin(slope);

            //not needed, but left anyway
            if(curX < 0 || curX > map->getNumGridRows() || curY < 0 || curY > map->getNumGridCols()) hit = 1;

            if(map->isGridValueObst( grid_map::Index(curX, curY)))
            {
              hit = 1;		//hit set to 1 if an obstacle is found
//              std::cout << "[newray.cpp@performSensingOperation] HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
            }

            if((long)curX == i && (long)curY == j)	//if the free cell is reached, set its value to 2 and stop the ray
            {
              map->setGridValue(dummy::Map::CellValue::VIST, i, j);
              counter++;
//              std::cout << "[newray.cpp@performSensingOperation] Cell scanned: " << i << " " << j  <<", value: "<< map->getGridValue(i, j) << std::endl;
              hit = 1;
            }
            u += 0.2;		//move forward along the ray
          }
        }
      }
    }
  }
  std::cout << "[newray.cpp@performSensingOperation] Totals cells scanned: " << counter << std::endl;
  return counter;
}

//convert the value along the y axis to the cartesian space in order to compute atan2
long NewRay::convertPoint(long y)
{
  return (NewRay::numGridRows - 1 - y);
}

long NewRay::convertPointPP(long y)
{
  return (NewRay::numPathPlanningGridRows - 1 - y);
}

int NewRay::getInformationGain(dummy::Map *map, double posX_meter, double posY_meter, int orientation, double FOV, int range)
{
  //int raycounter = 0;
  setGridToPathGridScale(map->getGridToPathGridScale());
  int counter = 0;	//count number of free cells that can be seen
  NewRay::numGridRows = map->getNumGridRows();

  // We are working on the navigation map with fine resolution
  long cell_i, cell_j;
  map->getGridIndex(posX_meter, posY_meter, cell_i, cell_j);
  range = range * gridToPathGridScale;

  //set the correct FOV orientation
  double startingPhi = orientation*PI/180 - FOV/2;
  double endingPhi = orientation*PI/180 + FOV/2;
  int add2pi = 0;

  if(startingPhi <= 0)
  {
    add2pi = 1;
    startingPhi = 2*PI + startingPhi;
    endingPhi = 2*PI + endingPhi;
  }

  if(endingPhi > 2*PI) add2pi = 1;

  //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

  //select the portion of map to be scanned
//  long minI = posX*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
//  long maxI = posX*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;
//  long minJ = posY*gridToPathGridScale + gridToPathGridScale/2 - range*gridToPathGridScale;
//  long maxJ = posY*gridToPathGridScale + gridToPathGridScale/2 + range*gridToPathGridScale;
  long minI = cell_i - range;
  long maxI = cell_j + range;
  long minJ = cell_j - range;
  long maxJ = cell_j + range;

  if(minI < 0) minI = 0;
  if(minJ < 0) minJ = 0;
  if(maxI > map->getNumGridRows()) maxI = map->getNumGridRows();
  if(maxJ > map->getNumGridCols()) maxJ = map->getNumGridCols();

  //scan the cells in the selected portion of the map
  for(long i = minI; i <= maxI; ++i)
  {
    for(long j = minJ; j <=maxJ; ++j)
    {
      double distance = sqrt((i - cell_i)*(i - cell_i) + (j - cell_j)*(j - cell_j));

      //if a cell is free and within range of the robot, generate the ray connecting the robot cell and the free cell
      if(map->getGridValue(i, j) == 0 && distance <= range)
      {
//        double curX = posX*gridToPathGridScale + gridToPathGridScale/2;		//starting position of the ray
//        double curY = posY*gridToPathGridScale + gridToPathGridScale/2;
//        double robotX = posX*gridToPathGridScale + gridToPathGridScale/2;		//position of the robot
//        double robotY = posY*gridToPathGridScale + gridToPathGridScale/2;

        double curX = cell_i;
        double curY = cell_j;
        double robotX = cell_i;
        double robotY = cell_j;

        double convertedI = NewRay::convertPoint(i);
        double convertedRX = NewRay::convertPoint(robotX);

        double slope = atan2(NewRay::convertPoint(i) - NewRay::convertPoint(robotX), j - robotY);	//calculate the slope of the ray with atan2

        if(slope <= 0 && add2pi == 0) slope = slope + 2*PI;
        if(add2pi == 1) slope = 2*PI + slope;		//needed in case of FOV spanning from negative to positive angle values

        //std::cout << std::endl << "StartingPhi: " << startingPhi << " EndingPhi: " << endingPhi <<std::endl;

        if(slope >= startingPhi && slope <= endingPhi)	//only cast the ray if it is inside the FOV of the robot
        {
          //raycounter++;
          //std::cout << "Inside loop, slope: " << slope  << " Cell: " << j << " " << i << std::endl;

          int hit = 0;			//set to 1 when obstacle is hit by ray or when the cell is reached in order to stop the ray
          double u = 0;			//current position along the ray
          while(hit == 0)		//scan the map along the ray until an ostacle is found or the considered cell is reached
          {

            //convert the position on the ray to cell coordinates to check the grid
            curY = robotY + 0.5 + u*cos(slope);
            curX = robotX + 0.5 - u*sin(slope);

            //not needed, but left anyway
            if(curX < 0 || curX > map->getNumGridRows() || curY < 0 || curY > map->getNumGridCols())
            {
              hit = 1;
              //break;
            }

            if(map->getGridValue((long)curX, (long)curY) == dummy::Map::CellValue::OBST)
            {
              hit = 1;		//hit set to 1 if an obstacle is found
              //std::cout << "HIT! cell: " << j << " " << i << " Hit point: " << curY << " " << curX << std::endl;
            }

            if((long)curX == i && (long)curY == j)	//if the free cell is reached, increase counter and stop the ray.
            {
              ++counter;
              //std::cout << "Cell scanned: " << (int)curY << " " << (int)curX << std::endl;
              hit = 1;
            }
            u += 0.2;		//move forward along the ray
          }
        }
      }
    }
  }
  //std::cout << "Number of rays: " << raycounter << std::endl;
  return counter;	//return the number of free cells

  // return this->informationGain;
}

void NewRay::setGridToPathGridScale(float value)
{
  gridToPathGridScale = value;
}

