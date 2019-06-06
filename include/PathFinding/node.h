#ifndef NODE_H
#define NODE_H
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <queue>
#include <string>
//#include <cdstdio>
#include <stdio.h>

using namespace std;

class Node {

  int dir = 8; // number of possible directions to go at any position
  // if dir==4
  // static int dx[dir]={1, 0, -1, 0};
  // static int dy[dir]={0, 1, 0, -1};
  // if dir==8
  int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  // current position
  int xPos;
  int yPos;
  // total distance already travelled to reach the node
  int level;
  // priority=level+remaining distance estimate
  int priority; // smaller: higher priority

public:
  Node(int xp, int yp, int d, int p);
  int getxPos() const;
  int getyPos() const;
  int getLevel() const;
  int getPriority() const;

  void updatePriority(const int &xDest, const int &yDest);

  // give better priority to going strait instead of diagonally
  void nextLevel(const int &i); // i: direction

  // Estimation function for the remaining distance to the goal.
  const int &estimate(const int &xDest, const int &yDest);
  // bool operator<(const Node & a,const Node & b);
  // bool operator<(const Node & a);
};

#endif