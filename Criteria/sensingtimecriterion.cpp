/*
 * Copyright 2015 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "Criteria/sensingtimecriterion.h"
#include "Criteria/criteriaName.h"
#include "newray.h"
# define PI           3.14159265358979323846  /* pi */
#include <iostream>

SensingTimeCriterion::SensingTimeCriterion(double weight):
    Criterion(SENSING_TIME, weight,true)
{

}


SensingTimeCriterion::~SensingTimeCriterion()
{

}

double SensingTimeCriterion::SensingTimeCriterion::evaluate(Pose &p,dummy::Map* map)
{
   NewRay ray;
    double sensingTime;
    double angle;
    int orientation = p.getOrientation();
    
    double startingPhi = orientation*PI/180 - (p.getFOV())/2;	
    double endingPhi = orientation*PI/180 + (p.getFOV())/2;
    int add2pi = 0;
  
    if(startingPhi <= 0) 
    {
    add2pi = 1;
    startingPhi = 2*PI + startingPhi;
    endingPhi = 2*PI + endingPhi;
    }
    
    
    
    
//    cout << " [sensingTimeCriterion.cpp@Evaluate] [posX, posY] = [" << p.getX() << "," << p.getY() << "]" << endl;
    double x_meter, y_meter;
    map->getPathPlanningPosition(x_meter , y_meter, p.getX(), p.getY());
//    cout << " [sensingTimeCriterion.cpp@Evaluate][2] [posX, posY] = [" << x_meter << "," << y_meter << "]" << endl;
    //sensingTime = ray.getSensingTime(map,p.getX(),p.getY(),p.getOrientation(),p.getFOV(),p.getRange());
    p.setScanAngles(ray.getSensingTime(map, x_meter, y_meter, p.getOrientation(), p.getFOV(), p.getRange()));
    double minPhi = p.getScanAngles().first;
    double maxPhi = p.getScanAngles().second;
   
    //std::cout << "minPhi " << p.getScanAngles().first << " maxPhi " << p.getScanAngles().second << std::endl;
    
    
    if(minPhi - startingPhi <= endingPhi - maxPhi) angle = (endingPhi - startingPhi - 2*(minPhi - startingPhi));
	else angle = (endingPhi - startingPhi - 2*(endingPhi - maxPhi));
	
	/*
	angle = (angle*180)/PI;

    if (angle <= 30){
	sensingTime = 0.2;
    }else if (angle >30 & angle <= 60){
	sensingTime = 0.4;
    }else if (angle > 60 & angle <=90){
	sensingTime = 0.6;
    }else if (angle > 90 & angle <= 120){
	sensingTime = 0.8;
    }else {
	sensingTime = 1;
    }
    
    */
    Criterion::insertEvaluation(p,angle);
    return angle;
}

/*
void SensingTimeCriterion::insertEvaluation(Pose& p, double value)
{
    insertEvaluation(p,value);
}
*/

