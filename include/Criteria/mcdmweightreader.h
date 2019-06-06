
#ifndef MCDMWEIGHTREADER_H
#define MCDMWEIGHTREADER_H

#include "weightmatrix.h"

class MCDMWeightReader {

public:
  WeightMatrix *getMatrix();

private:
  void listCasual(std::string str2);
};

#endif // MCDMWEIGHTREADER_H