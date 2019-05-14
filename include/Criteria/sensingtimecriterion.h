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

#ifndef SENSINGTIMECRITERION_H
#define SENSINGTIMECRITERION_H

#include "criterion.h"
#include "pose.h"
#include "map.h"

using namespace dummy;
class SensingTimeCriterion : public Criterion
{
public:
SensingTimeCriterion(double weight);
~SensingTimeCriterion();
double evaluate( Pose &p, dummy::Map* map);

//only for testing purpose
//void insertEvaluation(Pose &p, double value);
};

#endif // SENSINGTIMECRITERION_H
