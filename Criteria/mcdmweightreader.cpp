#include "Criteria/mcdmweightreader.h"
#include "Criteria/criteriaName.h"
#include <list>
#include <string.h>

using namespace std;

WeightMatrix *MCDMWeightReader::getMatrix(float w_criterion_1, float w_criterion_2, float w_criterion_3) {
  WeightMatrix *matrix = NULL;
  matrix = new WeightMatrix(3);
  // int numCriteria = 3;
  matrix->insertSingleCriterion(INFORMATION_GAIN, w_criterion_1, true);
  matrix->insertSingleCriterion(TRAVEL_DISTANCE, w_criterion_2, true);
  matrix->insertSingleCriterion(SENSING_TIME, w_criterion_3, true);
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
  matrix->insertCombinationWeight(list1, (w_criterion_1 + w_criterion_2 + 0.1));
  matrix->insertCombinationWeight(list2, (w_criterion_1 + w_criterion_3 + 0.1));
  matrix->insertCombinationWeight(list3, (w_criterion_2 + w_criterion_3 + 0.1));
  matrix->insertCombinationWeight(list4, (w_criterion_1 + w_criterion_2 + w_criterion_3));

  return matrix;
}
