#include "Criteria/weightmatrix.h"
#include "Criteria/criteriaName.h"
#include <iostream>
#include <sstream>

using namespace std;
/* mapping contains the criterion's name and his encoding
 * weights is a list (vector - for pratical purpose) of <encoding, weight> pairs
 */
WeightMatrix::WeightMatrix(int numOfCriteria)
    : // mapping(new unordered_map<string, string>()),
      // activeCriteria(new vector<pair<string, bool>>()),
      weights(new vector<unordered_map<string, double> *>()),
      lastInsertedCriteria(64),
      // mutex(new mutex()),
      numOfActiveCriteria(0) {

  // Create a row in the weight matrix for each weight cardinality
  for (int i = 0; i < numOfCriteria - 1; i++) {
    weights->push_back(new unordered_map<string, double>());
  }
}

WeightMatrix::~WeightMatrix() {
  // delete mutex;

  for (int i = weights->size() - 1; i >= 0; i--) {
    auto it = weights->begin();
    weights->erase(it);
  }
  delete weights;
  // activeCriteria->clear();
  // delete activeCriteria;
  // mapping->clear();
  // delete mapping;
}

/*insert the criterion's name and its encoding in mapping
 * unordered_map(hast_table) and
 * his encoding and his weight in weights vector */
void WeightMatrix::insertSingleCriterion(string name, double weight,
                                         bool active) {

  // mutex.lock();
  // increase the number of inserted criteria
  lastInsertedCriteria++;
  // encode the inserted criterion.
  // The coding of the single criteria is "A" for the first on,m "B" for the
  // second one, "C" for the third...
  // ldbg << "Criterion Names: " << name << endl;
  char tmp = (char)lastInsertedCriteria;
  // cout << tmp <<endl;
  stringstream ss;
  ss << tmp;
  string code;
  ss >> code;

  // ldbg << "Criterion Code: " << code << endl;
  // insert the entry in the mapping table
  std::pair<string, string> pair(name, code);
  mapping.insert(pair);
  std::pair<string, bool> pairActive(code, active);
  activeCriteria.push_back(pairActive);
  if (active)
    numOfActiveCriteria++;
  // insert the weight of the single criterion in the first row of the weight
  // matrix
  if (weights->size() > 0)
    std::pair<string, double> pair(code, weight);
  (weights->at(0))->emplace(code, weight);

  // weights->push_front(code,weight);
  // mutex.unlock();
}

void WeightMatrix::changeCriteriaActivation(const string &name, bool active) {
  // mutex.lock();
  // Get the encoding of the criterion
  string enc = mapping[name];
  // Get the actual state of the criterion activation
  int actualState;
  for (vector<pair<string, bool>>::iterator it = activeCriteria.begin();
       it != activeCriteria.end(); it++) {
    if ((*it).first == enc)
      actualState = (*it).second;
  }

  if (actualState == active) {
    // if the state should not change, return
    // mutex.unlock();
    return;
  }
  // If the state should change, override the past state of activation.
  pair<string, bool> tmp(enc, active);
  activeCriteria.push_back(tmp);
  if (active) // if the new state is a positive one, increase the number of
              // active criteria
    numOfActiveCriteria++;
  else // if the state is negative, decrease the number of active criteria.
    numOfActiveCriteria--;
  // mutex.unlock();
}

/*
 */
double WeightMatrix::getWeight(list<string> criteriaNames) {
  string enc = computeNamesEncoding(criteriaNames);
  double w = getWeight(enc);
  // cout << enc << " " << w << endl;
  if (w == 0) {
    // no encoding saved. I must compute the weight by summing
    // up every single weight.

    for (int i = 0; i < enc.length(); i++) {
      string e(to_string(enc.at(i)));
      w += getWeight(e);
    }
  }
  if (w > 1) // weights must belong to [0,1].
    w = 1;

  return w;
}

/* given an encoding of a criterion, return the weight associated to that
 * criterion
 */
double WeightMatrix::getWeight(const string &encoding) {
  // ldbg << "wights length = " << weights->length() << endl;
  int card = encoding.length();
  // mutex.lock();
  int numActiveCrit = numOfActiveCriteria;
  // mutex.unlock();
  // cout << card << " " << numActiveCrit << endl;
  if (card > numActiveCrit)
    return 1;
  if (card <= 0)
    return 0;
  // mutex.lock();
  // like a charm!
  double toRet = (weights->at(card - 1))->at(encoding);
  // mutex.unlock();
  return toRet;
}

int WeightMatrix::getNumOfActiveCriteria() {
  // mutex.lock();
  int toRet = numOfActiveCriteria;
  // mutex.unlock();
  return toRet;
}

/* return the encoding of the name searching for it in the mapping structure
 */
string WeightMatrix::getNameEncoding(string name) {
  // mutex.lock();
  string toRet = mapping[name];
  // mutex.unlock();
  return toRet;
}

/* given a list of criteria names (could be only one), copy the list of the
 * respective encoding in the enc list,
 * then sort it and append every single encoded criterion in the toRet string
 */
string WeightMatrix::computeNamesEncoding(list<string> criteriaNames) {
  // ldbg << "Criteria Names: " << criteriaNames << endl;
  // mutex.lock();
  if (criteriaNames.empty())
    return "";
  list<string> enc;
  list<string>::iterator it = criteriaNames.begin();
  for (it; it != criteriaNames.end(); ++it) {
    string tmp = mapping[*it];
    enc.emplace_back(tmp);
  }
  //    ldbg << "Encode before sorting: " << enc << endl;
  enc.sort();
  //    ldbg << "Encode after sorting: " << enc << endl;
  string toRet;
  for (it = enc.begin(); it != enc.end(); ++it) {
    string tmp = (*it);
    toRet.append(tmp);
  }
  // mutex.unlock();
  // ldbg << "Encoding " << toRet << endl;
  return toRet;
}

/*  given a list of criteria, call the proper function to encode the list in a
 * single string and then insert
 * it with the provided weight
 */
void WeightMatrix::insertCombinationWeight(list<string> criteriaNames,
                                           double weight) {

  insertCombinationWeight(computeNamesEncoding(criteriaNames), weight);
}

/*  insert in the weights structure the pair <enconding, weight> where enconding
 * is a string taking care of more
 * than one criterion.
 */
void WeightMatrix::insertCombinationWeight(const string &encoding,
                                           double weight) {
  int card = encoding.length();
  // mutex.lock();
  int mappingSize = mapping.size();
  // cout << card << " " << mappingSize << endl;
  // mutex.unlock();

  if (card >= mappingSize)
    return;
  if (card <= 0)
    return;
  // cout << weight << endl;   	correct weights here!!!
  // mutex.lock();
  (weights->at(card - 1))->emplace(encoding, weight);
  // mutex.unlock();
}

vector<string> WeightMatrix::getActiveCriteria() {
  // mutex.lock();
  vector<string> toRet;
  for (vector<pair<string, bool>>::iterator it = activeCriteria.begin();
       it != activeCriteria.end(); it++) {
    // k = endoding
    const string k = (*it).first;
    // if the status is active..
    if ((*it).second) {
      // i search for the name of the criterion with the same encoding in the
      // mapping structure
      string toApp;
      for (unordered_map<string, string>::iterator it2 = mapping.begin();
           it2 != mapping.end(); ++it2) {
        if ((*it2).second == k) {
          toApp = (*it2).first;
        }
      }
      toRet.push_back(toApp);
    }
  }

  // mutex.unlock();
  return toRet;
}

/* return the list of all criteria considered
 */
list<string> WeightMatrix::getKnownCriteria() {
  list<string> toRet;
  // mutex.lock();
  for (unordered_map<string, string>::iterator it = mapping.begin();
       it != mapping.end(); it++) {
    string tmp = (*it).first;
    toRet.emplace_back(tmp);
  }
  // mutex.unlock();
  return toRet;
}
