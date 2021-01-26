#ifndef INFORMATIONGAINCRITERION_H
#define INFORMATIONGAINCRITERION_H

#include "criterion.h"
#include "map.h"
#include "pose.h"
#include <vector>

class InformationGainCriterion : public Criterion {
public:
  InformationGainCriterion(double weight);
  virtual ~InformationGainCriterion();
  double evaluate(Pose &p, dummy::Map *map, ros::ServiceClient *path_client, vector<unordered_map<float,  std::pair<string, bayesian_topological_localisation::DistributionStamped>>> *mapping_time_belief, double *batteryTime, GridMap *belief_map, unordered_map<string,string> *mappingWaypoints, vector<bayesian_topological_localisation::DistributionStamped> *belief_topomaps);

private:
  void normalize(long minSensedX, int number);
  int *intersect(int p1x, int p1y, int p2x, int p2y, Pose &p);
};

#endif // INFORMATIONGAINCRITERION_H
