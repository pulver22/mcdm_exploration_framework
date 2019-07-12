#include "Criteria/traveldistancecriterion.h"
#include "map.h"
#include "mcdmfunction.h"
#include "newray.h"
#include <algorithm>
#include <iostream>
#include <iterator>
#define PI 3.14159265358979323846 /* pi */
#include <unistd.h>

using namespace std;
using namespace dummy;

void cleanPossibleDestination(std::list<Pose> *possibleDestinations, Pose &p);
void cleanPossibleDestination2(std::list<Pose> &possibleDestinations, Pose &p);
void cleanPossibleDestination(std::vector<Pose> *possibleDestinations, Pose &p);
void cleanPossibleDestination2(std::vector<Pose> &possibleDestinations,
                               Pose &p);
bool contains(std::list<Pose> &list, Pose &p);
bool contains(std::vector<Pose> &list, Pose &p);

int main(int argc, char **argv) {

  // Input : ./mcdm_online_exploration_ros
  // ./../Maps/map_RiccardoFreiburg_1m2.pgm 100 75 5 0 15 180 0.95 0.12
  // resolution x y orientation range centralAngle precision threshold

  ifstream infile;
  infile.open(argv[1]);
  int resolution = atoi(argv[2]);
  Map map = Map(infile, resolution);
  cout << "Map dimension: " << map.getNumGridCols() << " : "
       << map.getNumGridRows() << endl;
  // Pose initialPose = map.getRobotPosition();

  // i switched x and y because the map's orientation inside and outside
  // programs are different
  long initX = atoi(argv[4]);
  long initY = atoi(argv[3]);
  int initOrientation = atoi(argv[5]);
  double initFov = atoi(argv[7]);
  // FOV expressed in radiants
  initFov = initFov * PI / 180;
  int initRange = atoi(argv[6]);
  // percentage of coverage to be satisfied
  double precision = atof(argv[8]);
  double threshold = atof(argv[9]);
  // x,y,orientation,range,angle
  Pose initialPose = Pose(initX, initY, initOrientation, initRange, initFov);
  Pose target = initialPose;
  Pose previous = initialPose;
  long numConfiguration = 0;
  // testing
  // vector<pair<string,list<Pose> >> graph2;
  NewRay ray;
  MCDMFunction function;
  long sensedCells = 0;
  long newSensedCells = 0;
  long totalFreeCells = map.getTotalFreeCells();
  int count = 0;
  // int countBT;
  double travelledDistance = 0;
  unordered_map<string, int> visitedCell;
  vector<string> history;
  // history.push_back(function.getEncodedKey(target,1));
  // amount of time the robot should do nothing for scanning the environment (
  // final value expressed in second)
  // unsigned int microseconds = 5 * 1000 * 1000 ;
  // list<Pose> possibleDestinations;
  vector<Pose> possibleDestinations;
  possibleDestinations.push_back(initialPose);
  list<Pose> tabuList;
  bool noCandidatePosition = false;
  std::pair<Pose, double> result;

  while (sensedCells < precision * totalFreeCells) {

    long x = target.getX();
    long y = target.getY();
    int orientation = target.getOrientation();
    int range = target.getRange();
    double FOV = target.getFOV();
    string actualPose = function.getEncodedKey(target, 0);
    map.setCurrentPose(target);
    travelledDistance = travelledDistance + target.getDistance(previous);
    string encoding = to_string(target.getX()) + to_string(target.getY());
    visitedCell.emplace(encoding, 0);

    cout << "-----------------------------------------------------------------"
         << endl;
    cout << "Round : " << count << endl;
    newSensedCells = sensedCells +
                     ray.getInformationGain(map, x, y, orientation, FOV, range);
    cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells
         << endl;

    ray.performSensingOperation(map, x, y, orientation, FOV, range);
    ray.findCandidatePositions(map, x, y, orientation, FOV, range);
    vector<pair<long, long> > candidatePosition = ray.getCandidatePositions();
    ray.emptyCandidatePositions();

    if (candidatePosition.size() == 0) {
      // ATTENTION : doen't works, there is something for which sensed area is
      // wrong
      // if there are no other position to go from the actual pose
      cout << "No other candidate position" << endl;
      cout << "----- BACKTRACKING -----" << endl;

      // NEW METHOD BT
      cout << "No candidate position from here. Come back to the best previous "
              "position:"
           << endl;
      previous = target;

      EvaluationRecords *record2 =
          function.evaluateFrontiersVec(possibleDestinations, map, threshold);
      if (record2->size() != 0) {
        cout << "Many candidate positions" << endl;
        bool found = false;
        while (found == false) {
          result = function.selectNewPose(record2);
          if (count > 2240) {
          }
          target = result.first;

          if (contains(tabuList, target) == false) {
            tabuList.push_back(target);
            cout << "Found a new position" << endl;
            // cout << function.getEncodedKey(target, 0 )<< endl;
            found = true;
          } else {
            cout << "Position already visited, I need to search for a new one"
                 << endl;
            if (possibleDestinations.size() > 1) {
              cout << possibleDestinations.size() << endl;
              cleanPossibleDestination2(possibleDestinations, target);
              record2 = function.evaluateFrontiersVec(possibleDestinations, map,
                                                      threshold);
            } else {
              cout << "No other possible destination in my list" << endl;
              noCandidatePosition = true;
              delete record2;
              break;
            }
          }
        }

        if (noCandidatePosition == true)
          break;

        count = count + 1;
        numConfiguration++;
        delete record2;
      } else {
        noCandidatePosition = true;
        delete record2;
        break;
      }
      sensedCells = newSensedCells;

    } else {

      // need to convert from a <int,int pair> to a Pose with also
      // orientation,laser range and angle
      list<Pose> frontiers;
      vector<pair<long, long> >::iterator it = candidatePosition.begin();
      for (it; it != candidatePosition.end(); it++) {
        Pose p1 = Pose((*it).first, (*it).second, 0, range, FOV);
        Pose p2 = Pose((*it).first, (*it).second, 180, range, FOV);
        Pose p3 = Pose((*it).first, (*it).second, 90, range, FOV);
        Pose p4 = Pose((*it).first, (*it).second, 270, range, FOV);
        frontiers.push_back(p1);
        frontiers.push_back(p2);
        // frontiers.push_back(p3);
        // frontiers.push_back(p4);
        possibleDestinations.push_back(p1);
        possibleDestinations.push_back(p2);
      }

      // cout << "Candidate position: " << candidatePosition.size() << endl;
      // cout <<"Frontiers: "<<  frontiers.size() << endl;
      EvaluationRecords *record =
          function.evaluateFrontiers(frontiers, map, threshold);
      // cout << "Evaluation Record obtained" << endl;

      previous = target;
      // remove the actual pose from the possible ones

      if (record->size() != 0) {
        // if there are interesting positions to reach
        cout << "alive with possible position" << endl;
        result = function.selectNewPose(record);
        target = result.first;
        if (contains(tabuList, target) == false) {
          // if cell not already visited
          tabuList.push_back(target);
        } else {
          // if cell already visited
          cout << "Cell already visited" << endl;
          EvaluationRecords *record2 = function.evaluateFrontiersVec(
              possibleDestinations, map, threshold);
          if (record2->size() != 0) {
            cout << "[BT]No significative position reachable from here. Come "
                    "back to the best previous position:"
                 << endl;
            bool found = false;
            while (found == false) {
              result = function.selectNewPose(record2);
              target = result.first;
              cout << __LINE__ << " " << function.getEncodedKey(target, 0)
                   << endl;
              if (contains(tabuList, target) == false) {
                cout << "Found new position" << endl;
                tabuList.push_back(target);
                found = true;

              } else {
                cout << "Position already visited, I need to search for a new "
                        "one"
                     << endl;
                if (possibleDestinations.size() != 0) {
                  cleanPossibleDestination2(possibleDestinations, target);
                  record2 = function.evaluateFrontiersVec(possibleDestinations,
                                                          map, threshold);
                } else {
                  cout << "No other possible destination in my list" << endl;
                  noCandidatePosition = true;
                  delete record2;
                  break;
                }
              }
            }
            if (noCandidatePosition == true)
              break;
          }
        }
        count = count + 1;
        numConfiguration++;

      } else {
        // if there are no interesting candidate positions

        // NEW METHOD BT
        EvaluationRecords *record2 =
            function.evaluateFrontiersVec(possibleDestinations, map, threshold);
        if (record2->size() != 0) {
          cout << "[BT]No significative position reachable from here. Come "
                  "back to the best previous position:"
               << endl;
          bool found = false;
          while (found == false) {
            result = function.selectNewPose(record2);
            target = result.first;
            cout << __LINE__ << " " << function.getEncodedKey(target, 0)
                 << endl;
            if (contains(tabuList, target) == false) {
              cout << "trovata nuova posizione" << endl;
              tabuList.push_back(target);
              found = true;

            } else {
              cout << "Position already visited, I need to search for a new one"
                   << endl;
              if (possibleDestinations.size() != 0) {
                cleanPossibleDestination2(possibleDestinations, target);
                record2 = function.evaluateFrontiersVec(possibleDestinations,
                                                        map, threshold);
              } else {
                cout << "No other possible destination in my list" << endl;
                noCandidatePosition = true;
                delete record2;
                break;
              }
            }
          }
          if (noCandidatePosition == true)
            break;

          count = count + 1;
          numConfiguration++;
          delete record2;
        } else {
          noCandidatePosition = true;
          delete record2;
          break;
        }
      }

      sensedCells = newSensedCells;

      // NOTE: not requested for testing purpose
      // usleep(microseconds);

      // frontiers.clear();
      // candidatePosition.clear();
      delete record;
    }
  }

  map.drawVisitedCells(visitedCell, resolution);
  // map.printVisitedCells(history);

  if (noCandidatePosition == true) {
    cout << "-----------------------------------------------------------------"
         << endl;
    cout << "No other candidate position" << endl;
    cout << "Total cell visited :" << numConfiguration << endl;
    cout << "Total travelled distance (cells): " << travelledDistance << endl;
    cout << "FINAL : MAP NOT COMPLETELY EXPLORED! :( " << endl;
    cout << "-----------------------------------------------------------------"
         << endl;
  } else {
    // NEW METHOD
    cout << "-----------------------------------------------------------------"
         << endl;
    cout << "Total cell visited :" << numConfiguration << endl;
    cout << "Total travelled distance (cells): " << travelledDistance << endl;
    cout << "FINAL: MAP EXPLORED!" << endl;
    cout << "-----------------------------------------------------------------"
         << endl;
  }
}

void cleanPossibleDestination(std::vector<Pose> *possibleDestinations,
                              Pose &p) {
  MCDMFunction function;
  // cout<<"I remove "<< function.getEncodedKey(p,0) << endl;
  // cout << possibleDestinations->size() << endl;

  for (std::vector<Pose>::iterator it = possibleDestinations->begin();
       it != possibleDestinations->end(); it++) {
    if ((*it).isEqual(p)) {
      possibleDestinations->erase(it);
      break;
    } // else cout<< "not found" << endl;
  }
}

void cleanPossibleDestination2(std::vector<Pose> &possibleDestinations,
                               Pose &p) {
  MCDMFunction function;
  // cout<<"I remove "<< function.getEncodedKey(p,0) << endl;
  // cout << possibleDestinations->size() << endl;

  std::vector<Pose>::iterator findIter =
      std::find(possibleDestinations.begin(), possibleDestinations.end(), p);
  if (findIter != possibleDestinations.end()) {
    // cout << function.getEncodedKey(*findIter,0) << endl;
    possibleDestinations.erase(findIter);
  } else
    cout << "not found" << endl;
}

bool contains(std::vector<Pose> &list, Pose &p) {
  bool result = false;
  MCDMFunction function;

  std::vector<Pose>::iterator findIter = std::find(list.begin(), list.end(), p);
  if (findIter != list.end()) {
    // cout << "Found it: "<< function.getEncodedKey(p,0) <<endl;
    result = true;
  }

  return result;
}

bool contains(std::list<Pose> &list, Pose &p) {
  bool result = false;
  MCDMFunction function;

  std::list<Pose>::iterator findIter = std::find(list.begin(), list.end(), p);
  if (findIter != list.end()) {
    // cout << "Found it: "<< function.getEncodedKey(p,0) <<endl;
    result = true;
  }

  return result;
}