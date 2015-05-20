#include <cmath>
#include <iostream>
#include <stdlib.h>

# define PI           3.14159265358979323846  /* pi */

int worldMap[24][24]=
{
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,2,2,2,2,2,0,0,0,0,3,0,3,0,3,0,0,0,1},
  {1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,3,0,0,0,3,0,0,0,1},
  {1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,2,2,0,2,2,0,0,0,0,3,0,3,0,3,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,0,0,0,5,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

/*
void rotate90(double incX, double incY)
{
  double tempX = incX;
  double tempY = incY;
  
  incX = tempY;
  incY = - tempX;
}
*/

int main(int argc, char **argv)
{
  double mapX, mapY;			//coordinates in the map
  int posX = 6, posY = 5;		//starting position of the robot
  int orientation;			//orientation of the robot (0, 90, 180, 270 degrees)
  double FOV = PI/2;
  //int u;
  //double phi;
  int range = 5;			//range of the scanner
  
  if(argc > 1) orientation = atoi(argv[1]);
  else orientation = 0;
  
  //orientation = atoi(argv[1]);
  
  //double incX = sin(phi);
  //double incY = cos(phi);
  
  int counter = 0;			//count free cells that can be scanned
  
  for(double phi = (PI/2 - FOV/2); phi < (PI/2 + FOV/2); phi += (FOV/32))		//range through the circular sector
  {
    int hit = 0;			//set to 1 when obstacle is hit by ray
    int u = 0;				//current distance of the ray
    while(hit == 0 && u < range)	//scan the map along the ray until obstacle is found or end of range
    {
      
      //drives the ray according to orientation    X and Y are inverted in the formula in order to consider the euclidian space, not the array i(x) and j(y) indexes
      
      if(orientation == 0)
      {
      mapY = posY + u*sin(phi);
      mapX = posX + u*cos(phi);
      }
      
      if(orientation == 90)
      {
      mapY = posY + u*cos(phi);
      mapX = posX - u*sin(phi);
      }
      
      if(orientation == 180)
      {
      mapY = posY - u*sin(phi);
      mapX = posX - u*cos(phi);
      }
      
      if(orientation == 270)
      {
      mapY = posY - u*cos(phi);
      mapX = posX + u*sin(phi);
      }
      
      //rounding of the cell indexes
      
      if (mapY > 0) mapY += 0.5;
      else mapY -= 0.5;
	
      if (mapX > 0) mapX += 0.5;
      else mapX -= 0.5;
      
      std::cout << mapY << " " << mapX << std::endl;
      
      if(worldMap[(int)mapX][(int)mapY] > 0) hit = 1;		//hit set to 1 if an obstacle is found
      if(worldMap[(int)mapX][(int)mapY] == 0)			//free cell found
      {
	worldMap[(int)mapX][(int)mapY] = -1;			//set the cell to -1 so it won't be counted again in the future
	counter++;						//increase the count of free cells
      }
      ++u;							//move forward with the ray
    }
    std::cout << counter << std::endl;
  }
}