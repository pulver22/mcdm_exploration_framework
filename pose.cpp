#include "pose.h"
#include <cmath>
#include <iostream>
using namespace std;

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

bool Pose::operator==(const Pose& p)
{
    Pose p2 = p;
    //std::cout <<aX<<" "<<aY<<" "<<orientation<<" "<<FOV<<" "<<range<<std::endl;
    //std::cout<<p2.getX()<<" "<<p2.getY()<<" "<<p2.getOrientation()<<" "<<p2.getFOV()<<" "<<p2.getRange()<<std::endl;
    //std::cout<< "compare " << (aX == p2.getX()) <<" "<< (aY == p2.getY()) <<" "<< (orientation == p2.getOrientation()) <<" "<< (FOV == p2.getFOV()) <<" "<< (range == p2.getRange() )<<std::endl;
    return (aX == p2.getX()) && (aY == p2.getY()) && (orientation == p2.getOrientation()) && ((int)FOV == (int)(p2.getFOV())) && (range == p2.getRange());
}

void Pose::setScanAngles(std::pair< double, double > angles)
{
  //cout << angles.first << ":"<<angles.second << endl;
  scanAngles = angles;
}

std::pair<double, double> Pose::getScanAngles()
{
  return scanAngles;
}



