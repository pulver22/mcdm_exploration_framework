#ifndef MCDMWEIGHTREADER_H
#define MCDMWEIGHTREADER_H


#include "weightmatrix.h"



class MCDMWeightReader
{

public:

  WeightMatrix* getMatrix(float w_criterion_1, float w_criterion_2, float w_criterion_3);
  WeightMatrix* getMatrix(float w_criterion_1, float w_criterion_2, float w_criterion_3, float w_criterion_4);
  WeightMatrix* getMatrix(float w_criterion_1, float w_criterion_2, float w_criterion_3, float w_criterion_4, float w_criterion_5);


private:
  void listCasual(std::string str2);
};


#endif // MCDMWEIGHTREADER_H