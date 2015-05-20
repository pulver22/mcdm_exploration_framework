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

#include "graphpose.h"
#include <iostream>

using namespace std;

GraphPose::GraphPose()
{
}

GraphPose::~GraphPose()
{

}

void GraphPose::addPose(Pose &p,Pose &currentPose)
{
    
    Edge newEdge;
    newEdge.destination = p;
    newEdge.weight = currentPose.getDistance(p);
    
    Edge fakeEdge;
    fakeEdge.destination = currentPose;
    fakeEdge.weight = 0;
   
    // first pose of the tour
    if(graph2.size() == 0){
	string key = getEncodedKey(currentPose);
	std::list<Edge> list;
	list.push_back(fakeEdge);
	graph2.emplace(key,list);
	//cout << "inserimento della prima posizione effettuato" << endl;
	//int size = graph2->size();
	//cout << size << endl;
    }else{
	// add the new pose at the end of the graph
	string key2 = getEncodedKey(p);
	Edge fakeEdge2;
	fakeEdge2.destination =p;
	fakeEdge2.weight = 0;
	std::list<Edge> list;
	list.push_back(fakeEdge2);
	graph2.emplace(key2,list);
	//cout << "inserimento di una nuova posizione effettuato" << endl;
    }
    
   
    // add the new position to actual one's neighbors
    string key = getEncodedKey(currentPose);
    if (graph2.find(key) != graph2.end()) {
	(graph2.at(key)).push_back(newEdge);
    }
  
    
}

list< Edge >  GraphPose::getKnownDestination(Pose &p)
{
    string key = getEncodedKey(p);
    //list < Edge > toRet ;
    if (graph2.find(key) != graph2.end()) {
	return  graph2.at(key);
    }
}
    
std::unordered_map< string,list < Edge > > GraphPose::getGraph()
{
   return graph2;
}

string GraphPose::getEncodedKey(Pose& p)
{
    string key =  to_string(p.getX()) + "/" + to_string( p.getY()) + "/" + to_string( (int)p.getOrientation()) + "/"  + to_string(p.getRange()) + "/" + to_string((int)p.getFOV());
    return key;
}




