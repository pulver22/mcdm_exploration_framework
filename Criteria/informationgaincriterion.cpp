#include "Criteria/informationgaincriterion.h"
#include "Criteria/criteriaName.h"
#include <math.h>
#include "newray.h"
using namespace dummy;

InformationGainCriterion::InformationGainCriterion(double weight) :
    Criterion(INFORMATION_GAIN, weight,true)
{
}


InformationGainCriterion::~InformationGainCriterion()
{
}

double InformationGainCriterion::evaluate(Pose &p, dummy::Map &map)
{

    long px = p.getX();
    long py = p.getY();
    //float resolution = map.getResolution();
    //Get the orientation
    int orientation = p.getOrientation();
    int range = p.getRange();
    double angle = p.getFOV();
/*
    // Minimum and maximum coordinations sensed by the laser scan
    int minSensedX, maxSensedX;
    int minSensedY,maxSensedY;
    // intersection point between the laser sensor (at the edge) and the vertical/horizontal segment passing from 
    // the considered cell
    //int* intersection;    
    //Map as a bidimensional array (vector) starting
    vector<vector<int>> map2D = map.getMap2D();
    int maxValueY = map2D.size();
    int maxValueX = map2D[0].size();
    
    //area contained in the sensor cone
    int sensedArea;
    //area occupied by obstacles in the sensor cone
    int occupiedArea;
    //effective information gain
    int unExploredMap;
    
   
    
    //calcuate the sensed map based on the robot orientation
    // orientation == 90 means the robot is looking toward the upper border of the map
    if( orientation == 90 ){
	//minSensedX is the robot position minus the radius projection on the x axis; the same also for maxSensedX but with plus 
	//instead of minus
	minSensedX = px - p.getR() * cos(p.getPhi()/2);
	maxSensedX = px + p.getR() * cos(p.getPhi()/2);
	//minSensedY is the robot position minus the radius
	minSensedY = py - p.getR() ;
	//maxSensedY is the robot position, since the index grow towards the bottom
	maxSensedY = py;
	//normalize the sensed cell to not allow to sense outside the map edges
	normalize(maxSensedY, 0);
	normalize(maxSensedX,maxValueX);
	normalize(minSensedX,0);
	//count how many sensed cells are occupied by an obstacles
	for(int i=minSensedY  ; i< maxSensedY; i++){
	    for (int j = minSensedX; j< maxSensedX; j++){
		// if the considered cell is before the robot position (relating to the xaxis)
		if( j < px ){
		    // calculate the intersection point between the vertical segment passing for the cell and the radius at the left edge
		    int* intersection =intersect(i,j,minSensedX,minSensedY,p);
		    // if the j-coordinate of the considered cell is higher (y grow toward the bottom and x toward right) than the intersection's one, 
		    //it means that the cell is not insidie the sensed area, so break this loop and look for the following cell
		    if ( i > intersection[1]){
			continue;
		    }
		}else{
		    //as above but on the right side on the robot
		    int*intersection = intersect(i,j,maxSensedX,minSensedY,p);
		    if (i > intersection[1]){
		    continue;
		    }
		}
		// if the cell is inside the sensed area, we can increase our counter
		sensedArea++;
		//Hp: free cells are zero value
		if(map2D[i].at(j) = 1){
		    occupiedArea++;
		}
	    }
	}
    }else if ( orientation == 270){
	// case in which the robot is looking toward the bottom of the map
	minSensedX = px - p.getR() * cos(p.getPhi()/2);
	maxSensedX = px + p.getR() * cos(p.getPhi()/2);
	minSensedY = py ;
	maxSensedY = py + p.getR();
	normalize(maxSensedY, maxValueY);
	normalize(maxSensedX,maxValueX);
	normalize(minSensedX,0);
	for(int i=minSensedY  ; i< maxSensedY; i++){
	    for (int j = minSensedX; j< maxSensedX; j++){
		if( j < px ){
		    int* intersection = intersect(i,j,minSensedX,minSensedY,p);
		    if ( i < intersection[1]){
			continue;
		    }
		}else{
		    int* intersection = intersect(i,j,maxSensedX,minSensedY,p);
		    if (i < intersection[1]){
		    continue;
		    }
		}
		sensedArea++;
		//Hp: free cells are zero value
		if(map2D[i].at(j) =1){
		    occupiedArea++;
		}
	    }
	}
    }else if ( orientation == 0){
	// case in which the robot is looking toward east
	minSensedX = px;
	maxSensedX = px + p.getR();
	minSensedY = py - p.getR() * sin(p.getPhi()/2);
	maxSensedY = py + p.getR() * sin(p.getPhi()/2);
	normalize(maxSensedX,maxValueX);
	normalize(maxSensedY,0);
	normalize(minSensedY,maxValueY);
	for(int i=minSensedY  ; i< maxSensedY; i++){
	    for (int j = minSensedX; j< maxSensedX; j++){
		if( i < py ){
		    int* intersection = intersect(i,j,maxSensedX,minSensedY,p);
		    if ( i < intersection[1]){
			continue;
		    }
		}else{
		    int* intersection = intersect(i,j,maxSensedX,maxSensedY,p);
		    if (i < intersection[1]){
		    continue;
		    }
		}
		sensedArea++;
		//Hp: free cells are zero value
		if(map2D[i].at(j) =1){
		    occupiedArea++;
		}
	    }
	}
    }else if (orientation == 180){
	// case in which the robot is loooking toward west
	maxSensedX = px;
	minSensedX = px - p.getR() ;
	minSensedY = py - p.getR() * sin(p.getPhi()/2);
	maxSensedY = py + p.getR() * sin(p.getPhi()/2);
	for(int i=minSensedY  ; i< maxSensedY; i++){
	    for (int j = minSensedX; j< maxSensedX; j++){
		if( i < py ){
		    int* intersection = intersect(i,j,minSensedX,minSensedY,p);
		    if ( i < intersection[1]){
			continue;
		    }
		}else{
		    int* intersection = intersect(i,j,minSensedX,maxSensedY,p);
		    if (i < intersection[1]){
		    continue;
		    }
		}
		sensedArea++;
		//Hp: free cells are zero value
		if(map2D[i].at(j) =1){
		    occupiedArea++;
		}
	    }
	}
    }
    // the information gain is caluclated as the subtraction between the sensed area and the occupied one
    unExploredMap = sensedArea - occupiedArea;
    //insert in the evaluation record the pair <frontier,values>
    insertEvaluation(p,unExploredMap);
*/
    NewRay ray;
    //Map *map2 = &map;
    double unExploredMap=(double)ray.getInformationGain(map,px,py,orientation,angle,range);
    /*
    if (unExploredMap >= 0.5) {
	Criterion::insertEvaluation(p,unExploredMap);
    } else {
	cout<< "No other candidate position that guarantee information gain" << endl;
    }*/
    Criterion::insertEvaluation(p,unExploredMap);
    return unExploredMap;
}

/*
void InformationGainCriterion::normalize (long int minSensedX, int arg2)
{
    if(number == 0){
	if(position <= 0){
	position = 0;
	}
    }else{
	if(position >= number){
	    position = number;
	}
    }

}*/

/*
int* InformationGainCriterion::intersect(int p1x, int p1y, int p2x, int p2y, Pose p)
{
    float m1, m2, q1, q2;
    float mNum1, mNum2, mDen1, mDen2;
    float qNum1, qNum2, qDen1, qDen2;
    float py = p.getY();
    float px = p.getX();
    float intersectX, intersectY;
    int result[2];

    
        //Vertical or horizontal segment from the considered cell 
   if(p.getOrientation() == 90 || p.getOrientation() == 270){
	intersectX = p1x;
	intersectY =py + (intersectX - px)*(p2y - py) / (p2x - px);     
    } else if (p.getOrientation() == 0 || p.getOrientation() == 180){
	intersectY = p1y;
	intersectX = p2x + (intersectY - py) * (p2x -px) / (p2y - py);
    }

    result[1] = (int) intersectX;
    result[2] = (int) intersectY;
    
    
    //return the coordinations of the interesection point
    return (result);
}
*/


