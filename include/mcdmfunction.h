#ifndef MCDMFUNCTION_H
#define MCDMFUNCTION_H
#include "Criteria/criterion.h"
#include "Criteria/weightmatrix.h"
#include "evaluationrecords.h"
#include <utility>
#include "bayesian_topological_localisation/DistributionStamped.h"
#include <boost/numeric/ublas/symmetric.hpp>
#include <boost/numeric/ublas/io.hpp>

/**
 * This class implements the MCDM evaluation function
 * to evaluate the utility of the frontiers.
 */
using namespace dummy;
namespace bnu = boost::numeric::ublas;
class MCDMFunction {

public:
  MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, bool use_mcdm);
  MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, float w_criterion_4, bool use_mcdm);
  MCDMFunction(float w_criterion_1, float w_criterion_2, float w_criterion_3, float w_criterion_4, float w_criterion_5, bool use_mcdm);
  ~MCDMFunction();
  void evaluateFrontier(string currentRobotWayPoint, Pose &p, dummy::Map map,
                        ros::ServiceClient path_client,
                        vector<unordered_map<float,  std::pair<string, bayesian_topological_localisation::DistributionStamped>>> mapping_time_belief,
                        double batteryTime,
                        GridMap belief_map, unordered_map<string,string> mappingWaypoints,
                        prediction_tools tools,
                        std::unordered_map<string, double> distances_map);
  EvaluationRecords *evaluateFrontiers(string currentRobotWayPoint, const std::list<Pose> frontiers,
                                       dummy::Map map, double threshold,
                                       ros::ServiceClient path_client,
                                       vector<unordered_map<float,  std::pair<string, bayesian_topological_localisation::DistributionStamped>>> mapping_time_belief,
                                       double batteryTime,
                                       GridMap belief_map, unordered_map<string,string> mappingWaypoints, 
                                       prediction_tools tools,
                                       std::unordered_map<string, double> distances_map);
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
