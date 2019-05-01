#ifndef ASTAR_H
#define ASTAR_H
// FB - 201012256
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
//#include <cdstdio>
#include <stdio.h>
#include "map.h"
#include "PathFinding/node.h"

using namespace std;
using namespace dummy;


class Astar
{
private:
    const int dir=8; // number of possible directions to go at any position
    // if dir==4
    //static int dx[dir]={1, 0, -1, 0};
    //static int dy[dir]={0, 1, 0, -1};
    // if dir==8
    int dx[8]={1, 1, 0, -1, -1, -1, 0, 1};
    int dy[8]={0, 1, 1, 1, 0, -1, -1, -1};
    

public: 
    Astar();
    virtual ~Astar();
    double lenghtPath(string path );
    string pathFind( const int& xStart, const int& yStart, const int& xFinish, const int& yFinish, dummy::Map& originalMap );
    int getNumberOfTurning(string path);
   
};
#endif