#include "pose.h"
#include <cmath>


Pose::Pose(long aX,long aY, int orientation, int range, double FOV)
{
  Pose::aX = aX;
  Pose::aY = aY;
  Pose::orientation = orientation;
  Pose::range = range;
  Pose::FOV = FOV;
}

Pose::Pose()
{
  
}


Pose::~Pose()
{
  
}

double Pose::getDistance( Pose& pose)
{
  return std::sqrt((aX - pose.getX())*(aX - pose.getX()) + (aY - pose.getY())*(aY - pose.getY()));
}

long Pose::getX()
{
  return aX;
}

long int Pose::getY()
{
  return aY;
}

int Pose::getOrientation()
{
  return orientation;
}

int Pose::getRange()
{
  return range;
}

double Pose::getFOV()
{
  return FOV;
}

bool Pose::isEqual(Pose& p)
{
    if(aX == p.getX() & aY == p.getY() & orientation == p.getOrientation() & FOV == p.getFOV() & range == p.getRange()){
	return true;
    } else return false;
	
    

}

int Pose::getInformationGain()
{
  return informationGain;
}

void Pose::setInformationGain(int value)
{
  Pose::informationGain = value;
}





