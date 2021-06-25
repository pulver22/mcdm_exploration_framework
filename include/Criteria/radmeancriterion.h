//
// Created by pulver on 29/07/2019.
//
#ifndef RadMeanCriterion_H
#define RadMeanCriterion_H

#include "criterion.h"
#include "map.h"
#include "pose.h"
#include <vector>
#include "utils.h"

class RadMeanCriterion : public Criterion {
public:
  RadMeanCriterion(double weight);
  virtual ~RadMeanCriterion();
  double evaluate(string currentRobotWayPoint,
      Pose &p, dummy::Map map, ros::ServiceClient path_client,
      vector<unordered_map<float,
                           std::pair<string, bayesian_topological_localisation::
                                                 DistributionStamped>>>
          mapping_time_belief,
      double batteryTime, GridMap belief_map,
      unordered_map<string, string> mappingWaypoints, prediction_tools tools,
      std::unordered_map<string, double> distances_map);

private:
  void normalize(long minSensedX, int number);
  int *intersect(int p1x, int p1y, int p2x, int p2y, Pose &p);

  /**
   * We sum the entropy for all cells available in a given submap (5x5) around
   * the robot for all the belief maps
   *
   * @param p: the current pose of the robot
   * @param map: the reference to the map
   * @param rfid_tools: various utilities for RFID operation
   * @return the sum over the map of the entropy a tag is there
   */
  double evaluateEntropyOverBelief(Pose &p, GridMap *belief_map);

  /**
   * Calculate the entropy of the tag position over the map
   *
   * @param target: the robot pose
   * @param maxX: distance from one focal distance to the further edge
   * @param minX: distance from one focal distance to the clostest edge
   * @param tag_i: id of the tag emitting the signal
   */
  double getTotalEntropyEllipse(Pose target, double maxX, double minX,
                                int tag_i, GridMap *belief_map);
  /**
   * Calculate the entropy of the tag position over the map
   *
   * @param target: the robot pose
   * @param iterator: iterator over an ellipse
   * @param tag_i: id of the tag emitting the signal
   */
  double getTotalEntropyEllipse(Pose target, grid_map::EllipseIterator iterator,
                                int tag_i, GridMap *belief_map);

  std::string getTagLayerName(int tag_num);


  double evaluateEntropyTopologicalNode(
      Pose p, unordered_map<string, string> mappingWaypoints,
      vector<bayesian_topological_localisation::DistributionStamped>
          belief_topomaps);
  double evaluateEntropyTopologicalMap(
      vector<bayesian_topological_localisation::DistributionStamped>
          belief_topomaps);
  double computeKLTopologicalMap(
      vector<bayesian_topological_localisation::DistributionStamped>
          prior_distributions,
      vector<bayesian_topological_localisation::DistributionStamped>
          posterior_distributions);
  double computeEntropy(double likelihood);
  vector<
      std::pair<string, bayesian_topological_localisation::DistributionStamped>>
  findDistributionFromTime(double time,
                vector<unordered_map<
                    float, std::pair<string, bayesian_topological_localisation::
                                                 DistributionStamped>>>
                    mapping_time_belief);

  vector<vector<bayesian_topological_localisation::DistributionStamped>>
  getRFIDLikelihood(Pose p, prediction_tools tools, unordered_map<string, string> mappingWaypoints,
      vector<std::pair<string, bayesian_topological_localisation::DistributionStamped>> pf_update_distributions);

  vector<bayesian_topological_localisation::DistributionStamped>
      mergePriorLikelihood(prediction_tools tools, 
          vector<std::pair<string, bayesian_topological_localisation::DistributionStamped>> pf_update_distributions,
          vector<vector<bayesian_topological_localisation::DistributionStamped>> rfid_reading_likelihoods);


protected:
  double RadMeanInfoGain = 0.0;
  double tmp_belief = 0.0;
  float _free_space_val = 1.0; // or 1.0
  Utilities _utils;
};

#endif // RadMeanCriterion_H
