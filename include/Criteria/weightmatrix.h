#ifndef WEIGHTMATRIX_H
#define WEIGHTMATRIX_H

#include <cstring>
#include <functional>
#include <iterator>
#include <list>
#include <map>
#include <mutex>
#include <string.h>
#include <string>
#include <unordered_map>
#include <vector>

class WeightMatrix {
public:
  WeightMatrix(int numOfCriteria);
  virtual ~WeightMatrix();
  void insertSingleCriterion(std::string name, double weight, bool active);
  void insertCombinationWeight(std::list<std::string> criteriaNames,
                               double weight);
  void insertCombinationWeight(const std::string &encoding, double weight);
  double getWeight(std::list<std::string> criteriaNames);
  double getWeight(const std::string &encoding);
  std::string getNameEncoding(std::string name);
  std::string computeNamesEncoding(std::list<std::string> criteriaNames);
  std::list<std::string> getKnownCriteria();
  std::vector<std::string> getActiveCriteria();
  int getNumOfActiveCriteria();
  void changeCriteriaActivation(const std::string &name, bool active);

private:
  // This member maps a criterion name with its encoding
  std::unordered_map<std::string, std::string> mapping;

  // This is the double entrance table that contains all the weight.
  // - the index of the list indicate the cardinality of the weight combination.
  // - the Hash contains the pairs criteria_combination<->weight
  // NOTE: all the keys of the String must be sorted by lexicographic order.
  std::vector<std::unordered_map<std::string, double> *> *weights;
  std::vector<std::pair<std::string, bool> > activeCriteria;
  int numOfActiveCriteria;
  int lastInsertedCriteria;
  // std::mutex mutex;
};

#endif // WEIGHTMATRIX_H
