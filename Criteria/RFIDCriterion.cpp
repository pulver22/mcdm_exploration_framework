//
// Created by pulver on 29/07/2019.
//

#include "Criteria/RFIDCriterion.h"
#include "Criteria/criteriaName.h"
#include "Eigen/Eigen"
#include "newray.h"
#include "utils.h"
#include <math.h>

using namespace dummy;
using namespace grid_map;

RFIDCriterion::RFIDCriterion(double weight)
    : Criterion(RFID_READING, weight, true) {
  // minValue = 0.0;
}

RFIDCriterion::~RFIDCriterion() {}

double RFIDCriterion::evaluate(Pose &p, dummy::Map *map, ros::ServiceClient *path_client, double *batteryTime, GridMap *belief_map) {
  
  
  // 3) Calculate entropy around the cell
  this->RFIDInfoGain = evaluateEntropyOverBelief(p, belief_map);

  // 4) Calculate the KL divergence between prior and posterior distributions
  // this->RFIDInfoGain = evaluateKLDivergence(p, map, rfid_tools);

  Criterion::insertEvaluation(p, this->RFIDInfoGain);
  return this->RFIDInfoGain;
}

double RFIDCriterion::evaluateEntropyOverBelief(Pose &p, GridMap *belief_map) {
  float RFIDInfoGain = 0.0;
  double entropy_cell = 0.0;
  int buffer_size = 2;

  for (int tag_id = 0; tag_id < 10; tag_id++) {
    entropy_cell = getTotalEntropyEllipse(p, p.getRange(), -1.0, tag_id, belief_map);
    RFIDInfoGain += entropy_cell;
  }

  return RFIDInfoGain;
}


double RFIDCriterion::getTotalEntropyEllipse(Pose target, double maxX, double minX, int tag_i, GridMap *belief_map){
                                     //1.-  Get elipsoid iterator.
  // Antenna is at one of the focus of the ellipse with center at antennaX, antennaY, tilted antennaHeading .
  // http://www.softschools.com/math/calculus/finding_the_foci_of_an_ellipse/
  // if a is mayor axis and b is minor axis
  // a-c= minX
  // a+c= maxX
  // a = (maxX + minX)/2
  // c  = maxX/2 + minX
  // b  = sqrt(a^2-c^2)

  // mirror y axis!!!!
  double antennaX = target.getX();
  double antennaY = target.getY();
  double antennaHeading = target.getOrientation() * 3.14/180;

  double a =  (abs(maxX) + abs(minX))/2.0;
  double c =  (abs(maxX) - abs(minX))/2;
  double b = sqrt((a*a)-(c*c));
  double xc = antennaX + (c*cos(antennaHeading));
  double yc = antennaY + (c*sin(antennaHeading));

  Position center(xc, yc); // meters
  Length length(2*a, 2*b);
  grid_map::EllipseIterator el_iterator(*belief_map, center, length, antennaHeading);
  return getTotalEntropyEllipse(target, el_iterator, tag_i, belief_map);
}

double RFIDCriterion::getTotalEntropyEllipse(Pose target, grid_map::EllipseIterator iterator,
                                   int tag_i, GridMap *belief_map) {

  double total_entropy;
  Position point;
  double likelihood, neg_likelihood, log2_likelihood, log2_neg_likelihood = 0.0;

  std::string tagLayerName = getTagLayerName(tag_i);

  total_entropy = 0;
  for (iterator; !iterator.isPastEnd(); ++iterator) {
    belief_map->getPosition(*iterator, point);
    // check if is inside global map
    if (belief_map->isInside(point)) {
      // We don't add belief from positions considered obstacles...
      if (belief_map->atPosition("ref_map", point) == _free_space_val) {
        likelihood = belief_map->atPosition(tagLayerName, point);
        if (isnan(likelihood))
          likelihood = 0.0;
        neg_likelihood = 1 - likelihood;
        if (isnan(neg_likelihood))
          neg_likelihood = 0.0;

        log2_likelihood = log2(likelihood);
        if (isinf(log2_likelihood))
          log2_likelihood = 0.0;
        log2_neg_likelihood = log2(neg_likelihood);
        if (isinf(log2_neg_likelihood))
          log2_neg_likelihood = 0.0;
        // cout << " l: " << log2_likelihood << endl;
        // likelihood =
        // rfid_tools->rm.getBeliefMaps().atPosition(layerName,rel_point);
        total_entropy += -likelihood * log2_likelihood -
                         neg_likelihood * log2_neg_likelihood;
      }
    }
  }
  return total_entropy;
}

std::string RFIDCriterion::getTagLayerName(int tag_num) {
  return std::to_string(tag_num);
}