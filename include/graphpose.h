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

#ifndef GRAPHPOSE_H
#define GRAPHPOSE_H
#include "pose.h"
#include <vector>
#include <utility>
#include "edge.h"
#include <unordered_map>
#include <list>


using namespace std;

class GraphPose
{
   
    
public:
    GraphPose();
    ~GraphPose();
    //add the p pose in the graph of known pose as new state but also as reachable from a know position.
    // ATTENTION: the first element of the list containing the reachable state from a pose is occupied by a "fake" edge that connects the pose to itself with a zero weight
    void addPose(Pose &p, Pose &currentPose);
    //remove one pose from the graph
    void removePose(Pose &p);
    //return the vector contening the pose (and the travel cost express as distance) reachable from the one passed as variable 
   list<Edge >  getKnownDestination(Pose &p);
   unordered_map<string,list<Edge> >  getGraph();
   string getEncodedKey(Pose &p);
    
protected:
  Edge edge;
  unordered_map<string,list<Edge>>  graph2; 
 
 
    
};



#endif // GRAPHPOSE_H
