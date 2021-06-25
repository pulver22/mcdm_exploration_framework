#include "Criteria/mcdmweightreader.h"
#include "Criteria/criteriaName.h"
#include <list>
#include <string.h>
#include<iostream>

using namespace std;

WeightMatrix *MCDMWeightReader::getMatrix(float w_criterion_1, float w_criterion_2, float w_criterion_3) {
  WeightMatrix *matrix = NULL;
  // cout << w_criterion_1 << " " << w_criterion_2 << " " << w_criterion_3 << endl;
  matrix = new WeightMatrix(3);
  // int numCriteria = 3;
  bool to_use;
  to_use = (w_criterion_1 > 0.0) ? true : false;
  matrix->insertSingleCriterion(INFORMATION_GAIN, w_criterion_1, to_use);
  to_use = (w_criterion_2 > 0.0) ? true : false;
  matrix->insertSingleCriterion(TRAVEL_DISTANCE, w_criterion_2, to_use);
  to_use = (w_criterion_3 > 0.0) ? true : false;
  matrix->insertSingleCriterion(RAD_MEAN, w_criterion_3, to_use);
  string str1(INFORMATION_GAIN);
  string str2(TRAVEL_DISTANCE);
  string str3(RAD_MEAN);
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
  matrix->insertCombinationWeight(list4, 1.0);

  return matrix;
}

//WeightMatrix* MCDMWeightReader::getMatrix(float w_criterion_1, float w_criterion_2, float w_criterion_3)
//{
//  WeightMatrix *matrix = NULL;
//  matrix = new WeightMatrix(3);
//  //int numCriteria = 3;
//  matrix->insertSingleCriterion(INFORMATION_GAIN,0.2,true);
//  matrix->insertSingleCriterion(TRAVEL_DISTANCE,0.2,true);
//  matrix->insertSingleCriterion(RAD_MEAN,0.6,true);
//  string str1(INFORMATION_GAIN);
//  string str2(TRAVEL_DISTANCE);
//  string str3(RAD_MEAN);
//  list<string> list1 ;
//  list<string> list2 ;
//  list<string> list3 ;
//  list<string> list4;
//  list1.push_back(str1);
//  list1.push_back(str2);
//  list2.push_back(str1);
//  list2.push_back(str3);
//  list3.push_back(str2);
//  list3.push_back(str3);
//  list4.push_back(str1);
//  list4.push_back(str2);
//  list4.push_back(str3);
//  matrix->insertCombinationWeight(list1,0.5);
//  matrix->insertCombinationWeight(list2,0.9);
//  matrix->insertCombinationWeight(list3,0.9);
//  matrix->insertCombinationWeight(list4,1);
//
//  return matrix;
//}

WeightMatrix *MCDMWeightReader::getMatrix(float w_criterion_1, float w_criterion_2, float w_criterion_3, float w_criterion_4) {
  WeightMatrix *matrix = NULL;
  bool to_use;
  // cout << w_criterion_1 << " " << w_criterion_2 << " " << w_criterion_3 << " " << w_criterion_4 << endl;
  matrix = new WeightMatrix(4);
  // int numCriteria = 3;
  to_use = (w_criterion_1 > 0.0) ? true : false;
  matrix->insertSingleCriterion(INFORMATION_GAIN, w_criterion_1, to_use);
  to_use = (w_criterion_2 > 0.0) ? true : false;
  matrix->insertSingleCriterion(TRAVEL_DISTANCE, w_criterion_2, to_use);
  to_use = (w_criterion_3 > 0.0) ? true : false;
  matrix->insertSingleCriterion(RAD_MEAN, w_criterion_3, to_use);
  to_use = (w_criterion_4 > 0.0) ? true : false;
  matrix->insertSingleCriterion(RAD_VARIANCE, w_criterion_4, to_use);
  string str1(INFORMATION_GAIN);
  string str2(TRAVEL_DISTANCE);
  string str3(RAD_MEAN);
  string str4(RAD_VARIANCE);
  list<string> list1;  // IG, TD
  list1.push_back(str1);
  list1.push_back(str2);
  list<string> list2;  // IG, ST
  list2.push_back(str1);
  list2.push_back(str3);
  list<string> list3;  // IG, RFID
  list3.push_back(str1);
  list3.push_back(str4);
  list<string> list4;  // IG, TD, ST
  list4.push_back(str1);
  list4.push_back(str2);
  list4.push_back(str3);
  list<string> list5;  // IG, TD, RFID
  list5.push_back(str1);
  list5.push_back(str2);
  list5.push_back(str4);
  list<string> list6; // IG, ST, RFID
  list6.push_back(str1);
  list6.push_back(str3);
  list6.push_back(str4);
  list<string> list7; // TD, ST
  list7.push_back(str2);
  list7.push_back(str3);
  list<string> list8; // TD, RFID
  list8.push_back(str2);
  list8.push_back(str4);
  list<string> list9; // TD, ST, RFID
  list9.push_back(str2);
  list9.push_back(str3);
  list9.push_back(str4);
  list<string> list10; // ST, RFID
  list10.push_back(str3);
  list10.push_back(str4);
  list<string> list11; // IG, TD, ST, RFID
  list11.push_back(str1);
  list11.push_back(str2);
  list11.push_back(str3);
  list11.push_back(str4);
  matrix->insertCombinationWeight(list1, std::min((w_criterion_1 + w_criterion_2 + 0.1), 1.0));  // IG, TD
  matrix->insertCombinationWeight(list2, std::min((w_criterion_1 + w_criterion_3 + 0.1), 1.0));  // IG, ST
  matrix->insertCombinationWeight(list3, std::min((w_criterion_1 + w_criterion_4 + 0.1), 1.0));  // IG, RFID
  matrix->insertCombinationWeight(list4, std::min((w_criterion_1 + w_criterion_2 + w_criterion_3 + 0.1), 1.0));  // IG, TD, ST
  matrix->insertCombinationWeight(list5, std::min((w_criterion_1 + w_criterion_2 + w_criterion_4 + 0.1), 1.0));  // IG, TD, RFID
  matrix->insertCombinationWeight(list6, std::min((w_criterion_1 + w_criterion_3 + w_criterion_4 + 0.1), 1.0));  // IG, ST, RFID
  matrix->insertCombinationWeight(list7, std::min((w_criterion_2 + w_criterion_3 + 0.1), 1.0));  // TD, ST
  matrix->insertCombinationWeight(list8, std::min((w_criterion_2 + w_criterion_4 + 0.1), 1.0));  // TD, RFID
  matrix->insertCombinationWeight(list9, std::min((w_criterion_2 + w_criterion_3 + w_criterion_4 + 0.1), 1.0));  // TD, ST, RFID
  matrix->insertCombinationWeight(list10, std::min((w_criterion_3 + w_criterion_4 + 0.1), 1.0)); // ST, RFID
  matrix->insertCombinationWeight(list11, 1.0); // IG, TD, ST, RFID
  return matrix;
}


WeightMatrix *MCDMWeightReader::getMatrix(float w_criterion_1, float w_criterion_2, float w_criterion_3, float w_criterion_4, float w_criterion_5) {
  WeightMatrix *matrix = NULL;
  bool to_use;
  // cout << "WEIGHTS: " << w_criterion_1 << " " << w_criterion_2 << " " << w_criterion_3 << " " << w_criterion_4 << " " << w_criterion_5 << endl;
  matrix = new WeightMatrix(5);
  // int numCriteria = 3;
  to_use = (w_criterion_1 > 0.0) ? true : false;
  matrix->insertSingleCriterion(INFORMATION_GAIN, w_criterion_1, to_use);
  to_use = (w_criterion_2 > 0.0) ? true : false;
  matrix->insertSingleCriterion(TRAVEL_DISTANCE, w_criterion_2, to_use);
  to_use = (w_criterion_3 > 0.0) ? true : false;
  matrix->insertSingleCriterion(RAD_MEAN, w_criterion_3, to_use);
  to_use = (w_criterion_4 > 0.0) ? true : false;
  matrix->insertSingleCriterion(RAD_VARIANCE, w_criterion_4, to_use);
  to_use = (w_criterion_5 > 0.0) ? true : false;
  matrix->insertSingleCriterion(BATTERY_STATUS, w_criterion_5, to_use);
  string str1(INFORMATION_GAIN);
  string str2(TRAVEL_DISTANCE);
  string str3(RAD_MEAN);
  string str4(RAD_VARIANCE);
  string str5(BATTERY_STATUS);
  list<string> list1;  // IG, TD
  list1.push_back(str1);
  list1.push_back(str2);
  list<string> list2;  // IG, ST
  list2.push_back(str1);
  list2.push_back(str3);
  list<string> list3;  // IG, RFID
  list3.push_back(str1);
  list3.push_back(str4);
  list<string> list4;  // IG, TD, ST
  list4.push_back(str1);
  list4.push_back(str2);
  list4.push_back(str3);
  list<string> list5;  // IG, TD, RFID
  list5.push_back(str1);
  list5.push_back(str2);
  list5.push_back(str4);
  list<string> list6; // IG, ST, RFID
  list6.push_back(str1);
  list6.push_back(str3);
  list6.push_back(str4);
  list<string> list7; // TD, ST
  list7.push_back(str2);
  list7.push_back(str3);
  list<string> list8; // TD, RFID
  list8.push_back(str2);
  list8.push_back(str4);
  list<string> list9; // TD, ST, RFID
  list9.push_back(str2);
  list9.push_back(str3);
  list9.push_back(str4);
  list<string> list10; // ST, RFID
  list10.push_back(str3);
  list10.push_back(str4);
  list<string> list11; // IG, TD, ST, RFID
  list11.push_back(str1);
  list11.push_back(str2);
  list11.push_back(str3);
  list11.push_back(str4);
  list<string> list12; // IG, BS
  list12.push_back(str1);
  list12.push_back(str5);
  list<string> list13;  // TD, BS
  list13.push_back(str2);
  list13.push_back(str5);
  list<string> list14;  // ST, BS
  list14.push_back(str3);
  list14.push_back(str5);
  list<string> list15;  // RFID, BS
  list15.push_back(str4);
  list15.push_back(str5);
  list<string> list16;  // IG, TD, BS
  list16.push_back(str1);
  list16.push_back(str2);
  list16.push_back(str5);
  list<string> list17;  // IG, ST, BS
  list17.push_back(str1);
  list17.push_back(str3);
  list17.push_back(str5);
  list<string> list18;  // IG, RFID, BS
  list18.push_back(str1);
  list18.push_back(str4);
  list18.push_back(str5);
  list<string> list19;  // TD, ST, BS
  list19.push_back(str2);
  list19.push_back(str3);
  list19.push_back(str5);
  list<string> list20;  // TD, RFID, BS
  list20.push_back(str2);
  list20.push_back(str4);
  list20.push_back(str5);
  list<string> list21;  // ST, RFID, BS
  list21.push_back(str3);
  list21.push_back(str4);
  list21.push_back(str5);
  list<string> list22;  // IG, TD, ST, BS
  list22.push_back(str1);
  list22.push_back(str2);
  list22.push_back(str3);
  list22.push_back(str5);
  list<string> list23;  // IG, TD, RFID, BS
  list23.push_back(str1);
  list23.push_back(str2);
  list23.push_back(str4);
  list23.push_back(str5);
  list<string> list24;  // IG, ST, RFID, BS
  list24.push_back(str1);
  list24.push_back(str3);
  list24.push_back(str4);
  list24.push_back(str5);
  list<string> list25;  // TD, ST, RFID, BS
  list25.push_back(str2);
  list25.push_back(str3);
  list25.push_back(str4);
  list25.push_back(str5);
  list<string> list26;  // IG, TD, ST, RFID, BS
  list26.push_back(str1);
  list26.push_back(str2);
  list26.push_back(str3);
  list26.push_back(str4);
  list26.push_back(str5);
  matrix->insertCombinationWeight(list1, std::min((w_criterion_1 + w_criterion_2 + 0.1), 1.0));  // IG, TD
  matrix->insertCombinationWeight(list2, std::min((w_criterion_1 + w_criterion_3 + 0.1), 1.0));  // IG, ST
  matrix->insertCombinationWeight(list3, std::min((w_criterion_1 + w_criterion_4 + 0.1), 1.0));  // IG, RFID
  matrix->insertCombinationWeight(list4, std::min((w_criterion_1 + w_criterion_2 + w_criterion_3 + 0.1), 1.0));  // IG, TD, ST
  matrix->insertCombinationWeight(list5, std::min((w_criterion_1 + w_criterion_2 + w_criterion_4 + 0.1), 1.0));  // IG, TD, RFID
  matrix->insertCombinationWeight(list6, std::min((w_criterion_1 + w_criterion_3 + w_criterion_4 + 0.1), 1.0));  // IG, ST, RFID
  matrix->insertCombinationWeight(list7, std::min((w_criterion_2 + w_criterion_3 + 0.1), 1.0));  // TD, ST
  matrix->insertCombinationWeight(list8, std::min((w_criterion_2 + w_criterion_4 + 0.1), 1.0));  // TD, RFID
  matrix->insertCombinationWeight(list9, std::min((w_criterion_2 + w_criterion_3 + w_criterion_4 + 0.1), 1.0));  // TD, ST, RFID
  matrix->insertCombinationWeight(list10, std::min((w_criterion_3 + w_criterion_4 + 0.1), 1.0)); // ST, RFID
  matrix->insertCombinationWeight(list11, std::min((w_criterion_1 + w_criterion_2 + w_criterion_3 + w_criterion_4 + 0.1), 1.0)); // IG, TD, ST, RFID
  matrix->insertCombinationWeight(list12, std::min((w_criterion_1 + w_criterion_5 + 0.1), 1.0));  // IG, BS
  matrix->insertCombinationWeight(list13, std::min((w_criterion_2 + w_criterion_5 + 0.1), 1.0));  // TD, BS
  matrix->insertCombinationWeight(list14, std::min((w_criterion_3 + w_criterion_5 + 0.1), 1.0));  // ST, BS
  matrix->insertCombinationWeight(list15, std::min((w_criterion_4 + w_criterion_5 + 0.1), 1.0));  // RFID, BS
  matrix->insertCombinationWeight(list16, std::min((w_criterion_1 + w_criterion_2 + w_criterion_5 + 0.1), 1.0));  // IG, TD, BS
  matrix->insertCombinationWeight(list17, std::min((w_criterion_1 + w_criterion_3 + w_criterion_5 + 0.1), 1.0));  // IG, ST, BS
  matrix->insertCombinationWeight(list18, std::min((w_criterion_1 + w_criterion_4 + w_criterion_5 + 0.1), 1.0));  // IG, RFID, BS
  matrix->insertCombinationWeight(list19, std::min((w_criterion_2 + w_criterion_3 + w_criterion_5 + 0.1), 1.0));  // TD, ST, BS
  matrix->insertCombinationWeight(list20, std::min((w_criterion_2 + w_criterion_4 + w_criterion_5 + 0.1), 1.0));  // TD, RFID, BS
  matrix->insertCombinationWeight(list21, std::min((w_criterion_3 + w_criterion_4 + w_criterion_5 + 0.1), 1.0));  // ST, RFID, BS
  matrix->insertCombinationWeight(list22, std::min((w_criterion_1 + w_criterion_2 + w_criterion_3 + w_criterion_5 + 0.1), 1.0));  // IG, TD, ST, BS
  matrix->insertCombinationWeight(list23, std::min((w_criterion_1 + w_criterion_2 + w_criterion_4 + w_criterion_5 + 0.1), 1.0));  // IG, ST, RFID, BS
  matrix->insertCombinationWeight(list24, std::min((w_criterion_1 + w_criterion_3 + w_criterion_4 + w_criterion_5 + 0.1), 1.0));  // IG, ST, RFID, BS
  matrix->insertCombinationWeight(list25, std::min((w_criterion_2 + w_criterion_3 + w_criterion_4 + w_criterion_5 + 0.1), 1.0));  // TD, ST, RFID, BS
  matrix->insertCombinationWeight(list26, std::min((w_criterion_1 + w_criterion_2 + w_criterion_3 + w_criterion_4 + w_criterion_5 + 0.1), 1.0));  // IG, TD, ST, RFID, BS
  return matrix;
}