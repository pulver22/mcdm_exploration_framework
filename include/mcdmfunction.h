#ifndef MCDMFUNCTION_H
#define MCDMFUNCTION_H
#include "Criteria/criterion.h"
#include "Criteria/weightmatrix.h"
#include "evaluationrecords.h"
#include <utility>
#include "bayesian_topological_localisation/DistributionStamped.h"

/**
 * This class implements the MCDM evaluation function
 * to evaluate the utility of the frontiers.
 */
using namespace dummy;
class MCDMFunction {

public:
  MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, bool use_mcdm);
  MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, float w_criterion_4, bool use_mcdm);
  MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, float w_criterion_4, float w_criterion_5, bool use_mcdm);
  ~MCDMFunction();
  void evaluateFrontier(Pose &p, dummy::Map *map,
                        ros::ServiceClient *path_client,
                        double *batteryTime,
                        GridMap *belief_map, unordered_map<string,string> *mappingWaypoints,
                        vector<bayesian_topological_localisation::DistributionStamped> *belief_topomaps);
  EvaluationRecords *evaluateFrontiers(const std::list<Pose> *frontiers,
                                       dummy::Map *map, double threshold,
                                       ros::ServiceClient *path_client,
                                       double *batteryTime,
                                       GridMap *belief_map, unordered_map<string,string> *mappingWaypoints, 
                                       vector<bayesian_topological_localisation::DistributionStamped> *belief_topomaps);
  pair<Pose, double> selectNewPose(EvaluationRecords *evaluationRecords);
  string getEncodedKey(Pose &p, int value);

protected:
  Criterion *createCriterion(string name, double weight);
  unordered_map<string, Criterion *> criteria;
  vector<Criterion *> activeCriteria;
  WeightMatrix *matrix;
  bool use_mcdm = false;
  // mutex myMutex;
};
#endif // MCDMFUNCTION_H
