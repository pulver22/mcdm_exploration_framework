#include <vector>
#include "math.h"
#include "map.h"
#include "ray.h"

# define PI           3.14159265358979323846  /* pi */



Ray::Ray()
{
}


void Ray::findCandidatePositions(dummy::Map &map, long posX, long posY, int orientation, double FOV, int range)
{
  
  for(double phi = (PI/2 - FOV/2); phi < (PI/2 + FOV/2); phi += (FOV/64))		//range through the circular sector
  {
    int hit = 0;			//set to 1 when obstacle is hit by ray
    double u = 0;				//current distance of the ray
    mapX = posX;
    mapY = posY;
    double tempX, tempY;
    while(hit == 0 && u < range)	//scan the map along the ray until obstacle is found or end of range
    {
      
      tempX = mapX;
      tempY = mapY;
      
      //drives the ray according to orientation    X and Y are inverted in the formula in order to consider the euclidian space, not the array i(x) and j(y) indexes
      
      if(orientation == 0)
      {
      mapY = posY + 0.5 +  u*sin(phi);
      mapX = posX + 0.5 + u*cos(phi);
      }
      
      if(orientation == 90)
      {
      mapY = posY + 0.5 + u*cos(phi);
      mapX = posX + 0.5 - u*sin(phi);
      }
      
      if(orientation == 180)
      {
      mapY = posY + 0.5 - u*sin(phi);
      mapX = posX + 0.5 - u*cos(phi);
      }
      
      if(orientation == 270)
      {
      mapY = posY + 0.5 - u*cos(phi);
      mapX = posX + 0.5 + u*sin(phi);
      }
      
      if(mapX < 0 || mapX > numGridRows || mapY < 0 || mapY > numGridCols) hit = 1;
      
      //rounding of the cell indexes	TO BE REFINED
      
      /*
      
      if (mapY > 0) mapY += 0.5;
      else mapY -= 0.5;
	
      if (mapX > 0) mapX += 0.5;
      else mapX -= 0.5;
      
      */
      
      //std::cout << mapY << " " << mapX << std::endl;
      
      if((map.getGridValue((long)mapX,(long)mapY) != 2) || (u + 1 == range && map.getGridValue((long)mapX,(long)mapY) == 2 )) 
      {
	hit = 1;
	int newposition = 1;
	std::pair<long,long> temp = std::make_pair<long, long>((long)tempX,(long)tempY);
	
	for(long i = 0; i < Ray::edgePoints.size(); ++i)
	{
	  if(Ray::edgePoints.at(i) == temp) newposition = 0;
	}
	
	if(newposition == 1)
	{
	  Ray::edgePoints.push_back(temp);
	}
      }
      u += 0.5;
    }
  }
}

vector< std::pair<long, long> > Ray::getCandidatePositions()
{
  return Ray::edgePoints;
}

void Ray::emptyCandidatePositions()
{
  while(edgePoints.size() > 0)
  {
    edgePoints.pop_back();
  }
}


void Ray::setGrid(const dummy::Map& map)
{
  
  while(grid.size() > 0)
  {
    grid.pop_back();
  }
  
  Ray::numGridCols = map.getNumGridCols();
  Ray::numGridRows = map.getNumGridRows();
  
  
  for (int i = 0; i < numGridCols*numGridRows; ++i)
  {
    Ray::grid.push_back(map.getGridValue(i));
  }
}

int Ray::getInformationGain(const dummy::Map& map, long posX, long posY, int orientation, double FOV, int range)
{
  
  setGrid(map);
  
  
  int counter = 0;			//count free cells that can be scanned
  
  for(double phi = (PI/2 - FOV/2); phi < (PI/2 + FOV/2); phi += (FOV/64))		//range through the circular sector
  {
    int hit = 0;			//set to 1 when obstacle is hit by ray
    double u = 0;				//current distance of the ray
    while(hit == 0 && u < range)	//scan the map along the ray until obstacle is found or end of range
    {
      
      //drives the ray according to orientation    X and Y are inverted in the formula in order to consider the euclidian space, not the array i(x) and j(y) indexes
      
      if(orientation == 0)
      {
      mapY = posY + 0.5 + u*sin(phi);
      mapX = posX + 0.5 + u*cos(phi);
      }
      
      if(orientation == 90)
      {
      mapY = posY + 0.5 + u*cos(phi);
      mapX = posX + 0.5 - u*sin(phi);
      }
      
      if(orientation == 180)
      {
      mapY = posY + 0.5 - u*sin(phi);
      mapX = posX + 0.5 - u*cos(phi);
      }
      
      if(orientation == 270)
      {
      mapY = posY + 0.5 - u*cos(phi);
      mapX = posX + 0.5 + u*sin(phi);
      }
      
      if(mapX < 0 || mapX > numGridRows || mapY < 0 || mapY > numGridCols)hit = 1;
            
      //rounding of the cell indexes
      
      double decimalY;
      double decimalX;
      
      decimalY = mapY - (int)mapY;
      decimalX = mapX - (int)mapX;
      
      //std::cout << mapY << " " << mapX << std::endl;
      
      if(Ray::getGridValue((int)mapX, (int)mapY) == 1) hit = 1;		//hit set to 1 if an obstacle is found
      
      if(decimalY > 0.3 && decimalY < 0.7 && decimalX > 0.3 && decimalX < 0.7)
      {
      if(Ray::getGridValue((int)mapX, (int)mapY) == 0)			//free cell found
      {
	Ray::setGridValue((int)mapX, (int)mapY, -1);			//set the cell to -1 so it won't be counted again in the future
	counter++;						//increase the count of free cells
      }
      }
      u += 0.5;							//move forward with the ray
    }
  }
  
  return counter;
}
  

void Ray::setGridValue(long i, long j, int value)
{
  //if(value == 0 || value == 1 || value == 2)
  {
  Ray::grid[i*numGridCols + j] = value;
  }
}

int Ray::getGridValue(long i, long j)
{
  return Ray::grid[i*numGridCols + j];
}




void Ray::performSensingOperation(Map& map, long posX, long posY, int orientation, double FOV, int range)
{
  
  for(double phi = (PI/2 - FOV/2); phi < (PI/2 + FOV/2); phi += (FOV/64))		//range through the circular sector
  {
    int hit = 0;			//set to 1 when obstacle is hit by ray
    double u = 0;				//current distance of the ray
    while(hit == 0 && u < range)	//scan the map along the ray until obstacle is found or end of range
    {
      
      
      //drives the ray according to orientation    X and Y are inverted in the formula in order to consider the euclidian space, not the array i(x) and j(y) indexes
      
      if(orientation == 0)
      {
      mapY = posY + 0.5 + u*sin(phi);
      mapX = posX + 0.5 + u*cos(phi);
      }
      
      if(orientation == 90)
      {
      mapY = posY + 0.5 + u*cos(phi);
      mapX = posX + 0.5 - u*sin(phi);
      }
      
      if(orientation == 180)
      {
      mapY = posY + 0.5 - u*sin(phi);
      mapX = posX + 0.5 - u*cos(phi);
      }
      
      if(orientation == 270)
      {
      mapY = posY + 0.5 - u*cos(phi);
      mapX = posX + 0.5 + u*sin(phi);
      }
      
      if(mapX < 0 || mapX > numGridRows || mapY < 0 || mapY > numGridCols) hit = 1;
      
      //rounding of the cell indexes
      
      /*
      if (mapY > 0) mapY += 0.5;
      else mapY -= 0.5;
	
      if (mapX > 0) mapX += 0.5;
      else mapX -= 0.5;
      
      */
      
      double decimalY;
      double decimalX;
      
      decimalY = mapY - (int)mapY;
      decimalX = mapX - (int)mapX;
      
      
      //std::cout << phi << " " << u << " " << hit << " " << mapY << " " << mapX << std::endl;
      
      if(map.getGridValue((int)mapX,(int)mapY) == 1) hit = 1;		//hit set to 1 if an obstacle is found
      
      if(decimalY > 0.3 && decimalY < 0.7 && decimalX > 0.3 && decimalX < 0.7)
      {
      
      if(map.getGridValue((int)mapX,(int)mapY) == 0)			//free cell found
      {
	map.setGridValue(2, (int)mapX, (int)mapY);
	//std::cout << mapY << " " << mapX << std::endl;
      }
      }
      u += 0.5;							//move forward with the ray
    }
    //std::cout << counter << std::endl;

  }

}

