#ifndef EDGE_H
#define EDGE_H
#include "pose.h"

struct Edge {
  Pose destination;
  double weight;
};

#endif