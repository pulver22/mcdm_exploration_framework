//
// Created by pulver on 29/07/2019.
//
#ifndef RFIDCRITERION_H
#define RFIDCRITERION_H

#include "criterion.h"
#include "map.h"
#include "pose.h"
#include <vector>

class RFIDCriterion : public Criterion {
public:
  RFIDCriterion(double weight);
  virtual ~RFIDCriterion();
  double evaluate(Pose &p, dummy::Map *map, ros::ServiceClient *path_client, double *batteryTime, GridMap *belief_map, unordered_map<string,string> *mappingWaypoints, vector<bayesian_topological_localisation::DistributionStamped> *belief_topomaps);

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

  double evaluateEntropyTopologicalMap(Pose p,unordered_map<string,string> *mappingWaypoints, vector<bayesian_topological_localisation::DistributionStamped> *belief_topomaps);

protected:
  double RFIDInfoGain = 0.0;
  double tmp_belief = 0.0;
  float _free_space_val = 1.0;  // or 1.0
};

#endif // RFIDCRITERION_H
