#ifndef EVALUATIONRECORDS_H
#define EVALUATIONRECORDS_H

#include "pose.h"
#include <list>
#include <sstream>
#include <string.h>
#include <unordered_map>
#include <vector>

using namespace std;
/**
 * Data structure that contains, for each frontier, the utility
 * for the robot to reach the frontier.
 */
class EvaluationRecords {
public:
  /**
   * Constructor
   */
  EvaluationRecords();
  /**
   * Destructor.
   * The class destroy the frontiers. Be sure to create a copy of the
   * frontiers.
   */
  virtual ~EvaluationRecords();

  /**
   * This method insert an evaluation for the frontier. The method does not
   * make
   * a copy of the frontier, so be sure to create a copy from outside the
   * class and pass
   * the copy of the frontier as first parameter.
   * @param frontier the frontier
   * @param value the utility of the frontier
   */
  void putEvaluation(Pose &frontier, double value);
  /**
   * Method that return the utility value of the queried frontier
   * @param frontier the frontier for which we need the evaluation
   * @return the evaluation of the frontier.
   */
  double getEvaluation(Pose &frontier);
  /**
   * Method that return all the evaluation stored in this object.
   * Note that this method does not create a copy of the evaluation so,
   * when you delete this, you will delete also the evaluations.
   * @return the evaluations of all the frontiers.
   */
  unordered_map<string, double> getEvaluations();

  /**
   * Method that allow to query if a frontier is into the evaluation record
   * @param frontier the frontier to query
   * @return <b>true</b> if the frontier is contained into the object;
   * <b>false</b> otherwise.
   */

  bool contains(Pose &frontier);
  /**
   * Method that return the number of evaluated frontiers
   * @return the number of evaluations
   */
  int size();
  /**
   * Method that returns the list of all the frontiers that are evaluated.
   * Note that this method creates a copy of the frontiers, so you must
   * delete the list
   * and the frontiers when you do not need them.
   * @return the list of the evaluated frontiers.
   */
  list<Pose> getFrontiers();
  /**
   * Method that remove a frontier from the evaluation record.
   * @param frontier the frontier to remove.
   */
  void removeFrontier(Pose &frontier);

  /**
   * Calculate the encoding of a robot configuration
   *
   * @param p: the configuration of the robot as Pose object
   * @return the associated string encoding
   */
  Pose getPoseFromEncoding(string &encoding);

  /**
   * Normalise the evaluation between 0 and 1.
   */
  void normalize();
  string getEncodedKey(Pose &p);

  /**
   * Empty the evaluation record
   */
  void clear();

protected:
  unordered_map<string, double> evaluations;
  double maxValue, minValue;
};

#endif // EVALUATIONRECORDS_H
