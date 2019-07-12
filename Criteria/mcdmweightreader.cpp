#include "Criteria/mcdmweightreader.h"
#include "Criteria/criteriaName.h"
#include <list>
#include <string.h>

using namespace std;

WeightMatrix *MCDMWeightReader::getMatrix() {
  WeightMatrix *matrix = NULL;
  matrix = new WeightMatrix(3);
  // int numCriteria = 3;
  matrix->insertSingleCriterion(INFORMATION_GAIN, 1.0, true);
  matrix->insertSingleCriterion(TRAVEL_DISTANCE, 0.0, false);
  matrix->insertSingleCriterion(SENSING_TIME, 0.0, false);
  string str1(INFORMATION_GAIN);
  string str2(TRAVEL_DISTANCE);
  string str3(SENSING_TIME);
  list<string> list1;
  list<string> list2;
  list<string> list3;
  list<string> list4;
  list1.push_back(str1);
  list1.push_back(str2);
  list2.push_back(str1);
  list2.push_back(str3);
  list3.push_back(str2);
  list3.push_back(str3);
  list4.push_back(str1);
  list4.push_back(str2);
  list4.push_back(str3);
  matrix->insertCombinationWeight(list1, 1.0);
  matrix->insertCombinationWeight(list2, 1.0);
  matrix->insertCombinationWeight(list3, 0.0);
  matrix->insertCombinationWeight(list4, 1.0);

  return matrix;
}
