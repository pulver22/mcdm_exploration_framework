//
// Created by pulver on 29/07/2019.
//

#include "Criteria/RFIDCriterion.h"
#include "Criteria/criteriaName.h"
#include "Eigen/Eigen"
#include "newray.h"
#include "utils.h"
#include <algorithm> // std::find
#include <math.h>

#include "bayesian_topological_localisation/DistributionStamped.h"
#include "bayesian_topological_localisation/Predict.h"
#include "bayesian_topological_localisation/UpdatePriorLikelihoodObservation.h"
#include "rfid_grid_map/GetFakeBeliefMaps.h"

using namespace dummy;
using namespace grid_map;

RFIDCriterion::RFIDCriterion(double weight)
    : Criterion(RFID_READING, weight, false) { // true maximises
  // minValue = 0.0;
}

RFIDCriterion::~RFIDCriterion() {}

double RFIDCriterion::evaluate(string currentRobotWayPoint,
    Pose &p, dummy::Map *map, ros::ServiceClient *path_client,
    vector<unordered_map<float,
                         std::pair<string, bayesian_topological_localisation::
                                               DistributionStamped>>>
        *mapping_time_belief,
    double *batteryTime, GridMap *belief_map,
    unordered_map<string, string> *mappingWaypoints, prediction_tools *tools,
    std::unordered_map<string, double> *distances_map) {

  this->RFIDInfoGain = 0;
  double start = ros::Time::now().toSec();
  // Compute how long it takes to go to the destination waypoint and retrieve
  // the corresponding posterior belief maps
  // double path_len = Criterion::computeTopologicalDistance( p, path_client, mappingWaypoints);
  double path_len = Criterion::getPathLenFromMatrix(currentRobotWayPoint, p, distances_map, mappingWaypoints);
  double time = path_len / TRANSL_SPEED;
  // fesetround(FE_DOWNWARD);
  time = std::nearbyint(time);
  time = std::min(time, 50.0);
  // Obtain prior distribution at the correct time
  vector<
      std::pair<string, bayesian_topological_localisation::DistributionStamped>>
      pf_update_distributions = this->findDistributionFromTime(time, mapping_time_belief);

  // Obtain the fake likelihood distribution
  vector<vector<bayesian_topological_localisation::DistributionStamped>>
      rfid_reading_likelihoods = this->getRFIDLikelihood(
          p, tools, mappingWaypoints, &pf_update_distributions);

  // Compute posterior belief integrating update and correction step of the
  // particle filter
  // cout << "Pf prior: " << pf_update_distributions.size() << endl;
  // cout << "RFID Likelihood: " << rfid_reading_likelihoods.size() << endl;
  assert(pf_update_distributions.size() == rfid_reading_likelihoods.size());
  vector<bayesian_topological_localisation::DistributionStamped>
      posterior_distributions = this->mergePriorLikelihood(
          tools, &pf_update_distributions, &rfid_reading_likelihoods);

  // 1) Compute entropy on a single waypoint
  // this->RFIDInfoGain = evaluateEntropyTopologicalNode(p, mappingWaypoints,
                                                      // &posterior_distributions);
  // 2) Compute entropy on the entire map
  this->RFIDInfoGain =
  evaluateEntropyTopologicalMap(&posterior_distributions); 
  // cout << "Entropy
  // node: " << this->RFIDInfoGain << endl; 3) Compute KL-divergence between
  // prior and posterior distribution this->RFIDInfoGain =
  // computeKLTopologicalMap(&(tools->prior_distributions), &posterior_distributions);
  // cout << "   Entropy: " << this->RFIDInfoGain << endl;
  // cout << "RFIDCriterion: " << ros::Time::now().toSec() - start << endl;
  Criterion::insertEvaluation(p, this->RFIDInfoGain);
  return this->RFIDInfoGain;
}

double RFIDCriterion::evaluateEntropyOverBelief(Pose &p, GridMap *belief_map) {
  float RFIDInfoGain = 0.0;
  double entropy_cell = 0.0;
  int buffer_size = 2;
  std::vector<string> layers_name = belief_map->getLayers();
  // The layers_name vector contains "ref_map, X, Y" which are for not for
  // finding the tags. So we can remove their name to avoid checking this
  // layers.
  layers_name.erase(layers_name.begin(), layers_name.begin() + 3);
  // If there are no belief maps built (no signal received up to now),
  // the entropy is maximum
  if (layers_name.size() == 0) {
    RFIDInfoGain = 1.0; // default: max entropy
  }
  for (auto it = layers_name.begin(); it != layers_name.end(); it++) {
    int tag_id = std::stoi(*it); // convert string to int
    entropy_cell =
        getTotalEntropyEllipse(p, p.getRange(), -1.0, tag_id, belief_map);
    RFIDInfoGain += entropy_cell;
  }
  return RFIDInfoGain;
}

double RFIDCriterion::getTotalEntropyEllipse(Pose target, double maxX,
                                             double minX, int tag_i,
                                             GridMap *belief_map) {
  // 1.-  Get elipsoid iterator.
  // Antenna is at one of the focus of the ellipse with center at antennaX,
  // antennaY, tilted antennaHeading .
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
  double antennaHeading = target.getOrientation() * 3.14 / 180;

  double a = (abs(maxX) + abs(minX)) / 2.0;
  double c = (abs(maxX) - abs(minX)) / 2;
  double b = sqrt((a * a) - (c * c));
  double xc = antennaX + (c * cos(antennaHeading));
  double yc = antennaY + (c * sin(antennaHeading));
  Position center(xc, yc); // meters
  Length length(2 * a, 2 * b);
  grid_map::EllipseIterator el_iterator(*belief_map, center, length,
                                        antennaHeading);
  return getTotalEntropyEllipse(target, el_iterator, tag_i, belief_map);
}

double RFIDCriterion::getTotalEntropyEllipse(Pose target,
                                             grid_map::EllipseIterator iterator,
                                             int tag_i, GridMap *belief_map) {

  double total_entropy;
  Position point;
  double likelihood = 0.0;
  std::string tagLayerName = getTagLayerName(tag_i);

  total_entropy = 0;
  for (iterator; !iterator.isPastEnd(); ++iterator) {
    belief_map->getPosition(*iterator, point);
    // check if is inside global map
    if (belief_map->isInside(point)) {
      // We don't add belief from positions considered obstacles...
      if (belief_map->atPosition("ref_map", point) == _free_space_val) {
        likelihood = belief_map->atPosition(tagLayerName, point);
        total_entropy += this->computeEntropy(likelihood);
      }
    }
  }
  return total_entropy;
}

std::string RFIDCriterion::getTagLayerName(int tag_num) {
  return std::to_string(tag_num);
}

double RFIDCriterion::evaluateEntropyTopologicalNode(
    Pose p, unordered_map<string, string> *mappingWaypoints,
    vector<bayesian_topological_localisation::DistributionStamped>
        *belief_topomaps) {
  float RFIDInfoGain = 0.0;
  double likelihood = 0.0;
  EvaluationRecords record;
  string encoding = record.getEncodedKey(p);
  auto search = mappingWaypoints->find(encoding);
  string waypointName;
  // Look for waypoint name associated to the pose in exam
  if (search != mappingWaypoints->end()) {
    waypointName = search->second;
  } else {
    std::cout << "[RFIDCriterion.cpp@evaluateEntropyTopologicalMap] WayPoint "
                 "Not found\n";
  }

  if (belief_topomaps->size() != 0) {
    // For every belief map, look for the waypoint and access its value
    for (int tag_id = 0; tag_id < belief_topomaps->size(); tag_id++) {
      vector<string> nodes_list = belief_topomaps->at(tag_id).nodes;
      int index = 0;
      for (auto it = nodes_list.begin(); it != nodes_list.end(); it++) {
        if (*it == waypointName)
          break;
        else
          index++;
      }
      // cout << "[B]: " << accumulate(belief_topomaps->at(tag_id).values.begin(), belief_topomaps->at(tag_id).values.end(), 0.0) << endl;
      likelihood = belief_topomaps->at(tag_id).values[index];
      RFIDInfoGain += this->computeEntropy(likelihood);
    }
  }

  return RFIDInfoGain;
}

double RFIDCriterion::evaluateEntropyTopologicalMap(
    vector<bayesian_topological_localisation::DistributionStamped>
        *belief_topomaps) {
  double entropy = 0.0;
  double likelihood = 0.0;
  for (int tag_index = 0; tag_index < belief_topomaps->size(); tag_index++) {
    for (int node_index = 0;
         node_index < belief_topomaps->at(tag_index).values.size();
         node_index++) {
      likelihood = belief_topomaps->at(tag_index).values[node_index];
      entropy += this->computeEntropy(likelihood);
    }
  }

  return entropy;
}

double RFIDCriterion::computeKLTopologicalMap(
    vector<bayesian_topological_localisation::DistributionStamped>
        *prior_distributions,
    vector<bayesian_topological_localisation::DistributionStamped>
        *posterior_distributions) {
  double prior, posterior = 0.0;
  double tmp_KL = 0.0;
  double KL_div = 0.0;
  for (int tag_index = 0; tag_index < posterior_distributions->size();
       tag_index++) {
    for (int node_index = 0;
         node_index < posterior_distributions->at(tag_index).values.size();
         node_index++) {
      prior = prior_distributions->at(tag_index).values[node_index];
      posterior = posterior_distributions->at(tag_index).values[node_index];
      tmp_KL = posterior * log2(posterior / prior);
      if (isnan(tmp_KL) or isinf(tmp_KL))
        tmp_KL = 0;
      KL_div += tmp_KL;
    }
  }
  return KL_div;
}

double RFIDCriterion::computeEntropy(double likelihood) {
  double neg_likelihood, log2_likelihood, log2_neg_likelihood = 0.0;
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

  return -likelihood * log2_likelihood - neg_likelihood * log2_neg_likelihood;
}

vector<
    std::pair<string, bayesian_topological_localisation::DistributionStamped>>
RFIDCriterion::findDistributionFromTime(
    double time,
    vector<unordered_map<float,
                         std::pair<string, bayesian_topological_localisation::
                                               DistributionStamped>>>
        *mapping_time_belief) {

  vector<
      std::pair<string, bayesian_topological_localisation::DistributionStamped>>
      posterior_distributions;
  for (int tag_index = 0; tag_index < mapping_time_belief->size();
       tag_index++) {
    auto search = mapping_time_belief->at(tag_index).find(time);
    if (search != mapping_time_belief->at(tag_index).end()) {
      // Save the pair (estimated_node, distribution)
      posterior_distributions.push_back(
          std::make_pair(search->second.first, search->second.second));
    }
    // else cout << "Not found time = " << time << endl;
  }

  return posterior_distributions;
}

vector<vector<bayesian_topological_localisation::DistributionStamped>>
RFIDCriterion::getRFIDLikelihood(
    Pose p, prediction_tools *tools,
    unordered_map<string, string> *mappingWaypoints,
    vector<std::pair<string,
                     bayesian_topological_localisation::DistributionStamped>>
        *pf_update_distributions) {
  vector<vector<bayesian_topological_localisation::DistributionStamped>>
      rfid_reading_likelihoods;
  bayesian_topological_localisation::DistributionStamped tmp_belief_topo;
  rfid_grid_map::GetFakeBeliefMaps belief_map_srv;
  grid_map_msgs::GridMap belief_map_msg;
  string waypoint_encoding;
  EvaluationRecords record;
  Pose tmp_pose;
  GridMapRosConverter converter;
  grid_map::GridMap belief_map;
  // For all the registered tags, call the dedicated server for obtaining a
  // future fake likelihood readings
  belief_map_srv.request.antenna_x = p.getX();
  belief_map_srv.request.antenna_y = p.getY();
  belief_map_srv.request.antenna_h = p.getOrientation();

  double start = ros::Time::now().toSec();
  for (int tag_id = 0; tag_id < tools->prior_distributions.size();
       tag_id++) { // for every tag
    // Create vector of distributions for the current tag
    vector<bayesian_topological_localisation::DistributionStamped> tmp_distributions;
    for (auto it = mappingWaypoints->begin(); it != mappingWaypoints->end();
         it++) {
      // NOTE: mappingWaypoints contains pair <encoding, waypoint_name>
      if (pf_update_distributions->at(tag_id).first == it->second)
        waypoint_encoding = it->first;
    }
    tmp_pose = record.getPoseFromEncoding(waypoint_encoding);
    belief_map_srv.request.tag_x = tmp_pose.getX();
    belief_map_srv.request.tag_y = tmp_pose.getY();
    for (auto& x : tools->radarmodel_fake_reading_srv_list){
      if (x.call(belief_map_srv)) {
        belief_map_msg = belief_map_srv.response.rfid_maps;
        converter.fromMessage(belief_map_msg, belief_map);
        // cout << belief_map["42"].sum() << endl;
        // Convert from gridmap to topological
        // NOTE: There should be only one layer called '42'
        std::vector<string> layers_name = belief_map.getLayers();
        tmp_belief_topo = _utils.convertGridBeliefMapToTopoMap(
            &belief_map, &(tools->topoMap), mappingWaypoints, layers_name[0], 1.0);
        tmp_distributions.push_back(tmp_belief_topo);

        // cout << "[C]: " << accumulate(tmp_belief_topo.values.begin(), tmp_belief_topo.values.end(), 0.0) << endl;
      }else cout << "[ERROR]Service NOT CALLED" << endl;
    }
    rfid_reading_likelihoods.push_back(tmp_distributions);
  }
  // cout << "Correction: " << ros::Time::now().toSec() - start << endl;
  // cout << "rfid_reading_likelihoods: " << rfid_reading_likelihoods.size() << endl;

  return rfid_reading_likelihoods;
}

vector<bayesian_topological_localisation::DistributionStamped>
RFIDCriterion::mergePriorLikelihood(
    prediction_tools *tools,
    vector<std::pair<string,
                     bayesian_topological_localisation::DistributionStamped>>
        *pf_update_distributions,
    vector<vector<bayesian_topological_localisation::DistributionStamped>>
        *rfid_reading_likelihoods) {

  vector<bayesian_topological_localisation::DistributionStamped>
      posterior_distributions;
  bayesian_topological_localisation::UpdatePriorLikelihoodObservation
      update_srv;
  // double start = ros::Time::now().toSec();
  for (int tag_id = 0; tag_id < tools->pf_stateless_update_srv_list.size();
       tag_id++) {
    // NOTE: each tag has one associated distribution for each antenna present
    // We need to recursively update the prior with each new reading
    bayesian_topological_localisation::DistributionStamped 
        tmp_distribution = pf_update_distributions->at(tag_id).second;
    for(auto& rfid_likelihood : rfid_reading_likelihoods->at(tag_id)){
      update_srv.request.prior = tmp_distribution;
      update_srv.request.likelihood = rfid_likelihood;
      // Integrate the RFID likelihood with the distribution coming from the PF
      if (tools->pf_stateless_update_srv_list.at(tag_id).call(update_srv)) {
        if (update_srv.response.success){
          // Set the prior with the new posterior
          tmp_distribution = update_srv.response.current_prob_dist;
        } 
        else
          cout << "[RFIDCriterion.cpp@mergePriorLikelihood] An error occured" << endl;
      }
    }
    posterior_distributions.push_back(tmp_distribution);
  }
  // cout << "Merge: " << ros::Time::now().toSec() - start << endl;
  return posterior_distributions;
}