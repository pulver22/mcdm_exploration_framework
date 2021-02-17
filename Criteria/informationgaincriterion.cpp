#include "Criteria/informationgaincriterion.h"
#include "Criteria/criteriaName.h"
#include "newray.h"
#include <math.h>
using namespace dummy;

InformationGainCriterion::InformationGainCriterion(double weight)
    : Criterion(INFORMATION_GAIN, weight, true) {}

InformationGainCriterion::~InformationGainCriterion() {}

double InformationGainCriterion::evaluate(string currentRobotWayPoint,
    Pose &p, dummy::Map map, ros::ServiceClient path_client,
    vector<unordered_map<float,
                         std::pair<string, bayesian_topological_localisation::
                                               DistributionStamped>>>
        mapping_time_belief,
    double batteryTime, GridMap belief_map,
    unordered_map<string, string> mappingWaypoints, prediction_tools tools,
    std::unordered_map<string, double> distances_map) {

  float px = p.getX();
  float py = p.getY();
  // Get the orientation
  float orientation = p.getOrientation();
  int range = p.getRange();
  double angle = p.getFOV();
  NewRay ray;
  double unExploredMap = 0;
  unExploredMap = map.getInformationGain(px, py, orientation, angle, range);
  Criterion::insertEvaluation(p, unExploredMap);
  return unExploredMap;
}

