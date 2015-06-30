#ifndef POSE_H
#define POSE_H

#include <utility>



class Pose 
{
public:
  Pose();
  Pose(long aX, long aY, int orientation, int range, double FOV);
  virtual ~Pose();
  double getDistance( Pose &pose);   
  long getX();
  long getY();
  int getOrientation();
  int getRange();
  double getFOV();
  bool isEqual(Pose &p);
  int getInformationGain();
  void setInformationGain(int value);
  bool operator==(const Pose& p);
  void setScanAngles(std::pair<double,double> angles);
  std::pair<double, double> getScanAngles();
  
  
protected:
  long aX, aY;		// x and y coordinates of the cell
  int orientation;	// orientation theta of th robot
  int range;		// radius of the sensing operation 
  double FOV;		// central angle of the sensing operation
  int informationGain;
   std::pair<double,double> scanAngles;
  
};


#endif