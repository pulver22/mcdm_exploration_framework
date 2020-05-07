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

#ifndef CRITERION_H
#define CRITERION_H
#include <unordered_map>
#include <string>
#include "pose.h"
#include "map.h"
// #include "RadarModel.hpp"
#include "constants.h"

//using namespace import_map;
using namespace std;
using namespace dummy;
class Criterion
{
    public:
	//Constructor and destructor
	Criterion();
	Criterion( string name, double weight,bool highGood);
	~Criterion();

	//Other methods
	virtual double evaluate( Pose &p, dummy::Map *map, ros::ServiceClient* path_client, double *batteryTime, GridMap *belief_map) {};
	double getEvaluation(Pose &p) const;
	void insertEvaluation(Pose &p, double value);
	void clean();
	void normalize();

	//Setters and getters
	string getName() ;
	double getWeight() ;
	void setName( string name);
	void setWeight(double weight);
    private:
	void normalizeHighGood();
	void normalizeLowGood();
	string getEncodedKey(Pose &p);

  protected:
	string name;
	double weight = 0.0;
	bool highGood;
	double maxValue, minValue;

  private:
	unordered_map<string , double> evaluation;
};

#endif // CRITERION_H
