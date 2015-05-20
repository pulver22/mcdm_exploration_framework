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

#include "sensingtimecriterion.h"
#include "criteriaName.h"
#include "newray.h"


SensingTimeCriterion::SensingTimeCriterion(double weight):
    Criterion(SENSING_TIME, weight,true)
{

}


SensingTimeCriterion::~SensingTimeCriterion()
{

}

double SensingTimeCriterion::SensingTimeCriterion::evaluate(Pose &p,dummy::Map &map)
{
    NewRay ray;
    double sensingTime;
    
    float phi = p.getFOV();
    if (phi <= 30){
	sensingTime = 0.2;
    }else if (phi >30 & phi <= 60){
	sensingTime = 0.4;
    }else if (phi > 60 & phi <=90){
	sensingTime = 0.6;
    }else if (phi > 90 & phi <= 120){
	sensingTime = 0.8;
    }else {
	sensingTime = 1;
    }
    //sensingTime = ray.getSensingTime(map,p.getX(),p.getY(),p.getOrientation(),p.getFOV(),p.getRange());
    Criterion::insertEvaluation(p,sensingTime);
    return sensingTime;
}

/*
void SensingTimeCriterion::insertEvaluation(Pose& p, double value)
{
    insertEvaluation(p,value);
}
*/

