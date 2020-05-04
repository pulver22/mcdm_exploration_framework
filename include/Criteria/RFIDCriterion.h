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
  double evaluate(Pose &p, dummy::Map *map, ros::ServiceClient *path_client, RFID_tools *rfid_tools, double *batteryTime);

private:
  void normalize(long minSensedX, int number);
  int *intersect(int p1x, int p1y, int p2x, int p2y, Pose &p);

  /**
   * Method using the ellipse map leading to results identical to
   * informationGain. All the cells in the ellipse have the same value. NOTE:
   * DEPRECATED
   *
   * @param p: the current pose of the robot
   * @param map: the reference to the map
   * @return the number of cells with RFID information
   */
  double evaluateUniformEllipse(Pose &p, dummy::Map *map);

  /**
   * We sum all the pixels intensity (belief) for all the belief maps available
   * in a given submap (5x5) around the robot
   *
   * @param p: the current pose of the robot
   * @param map: the reference to the map
   * @param rfid_tools: various utilities for RFID operation
   * @return the sum over the map of the likelihood a tag is there
   */
  double evaluateSumOverBelief(Pose &p, dummy::Map *map,
                               RFID_tools *rfid_tools);

  /**
   * We sum the entropy for all cells available in a given submap (5x5) around
   * the robot for all the belief maps
   *
   * @param p: the current pose of the robot
   * @param map: the reference to the map
   * @param rfid_tools: various utilities for RFID operation
   * @return the sum over the map of the entropy a tag is there
   */
  double evaluateEntropyOverBelief(Pose &p, dummy::Map *map,
                                   RFID_tools *rfid_tools);

    /**
   * We sum the KL divergence for all cells available in a given submap (5x5) around
   * the robot for all the belief maps
   *
   * @param p: the current pose of the robot
   * @param map: the reference to the map
   * @param rfid_tools: various utilities for RFID operation
   * @return the sum over the map of the entropy a tag is there
   */
  double evaluateKLDivergence(Pose &p, dummy::Map *map,
                                   RFID_tools *rfid_tools);

protected:
  double RFIDInfoGain = 0.0;
  double tmp_belief = 0.0;
};

#endif // RFIDCRITERION_H
