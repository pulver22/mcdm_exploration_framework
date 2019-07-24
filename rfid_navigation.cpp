#include "Criteria/traveldistancecriterion.h"
#include "PathFinding/astar.h"
#include "map.h"
#include "mcdmfunction.h"
#include "newray.h"
#include "navigation_utilities.cpp"
#include "radio_models/propagationModel.cpp"
#include <algorithm>
#include <iostream>
#include <iterator>

#define _USE_MATH_DEFINES

#include "math.h"
#include <ctime>
#include <time.h>
#include <unistd.h>
#include "RFIDGridmap.h"
#include "movebasegoal.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// mfc ...
#include <ros/console.h>
#include "record_ros/record.h"
#include "record_ros/String_cmd.h"
// mfc ...

using namespace std;
using namespace dummy;

/**
 *
 *  Forked from rfid_navigation.cpp
 *
 * This should update and maintain a belief about the rfid detections.
 *
 */


// .............................................................................
// headers .....................................................................
// .............................................................................

// ROS varies
bool move(float x, float y, float orientation, float time_travel,
          list<Pose> *tabuList,
          std::list<std::pair<float, float> > *posToEsclude);

void update_callback(const map_msgs::OccupancyGridUpdateConstPtr &msg);

void grid_callback(const nav_msgs::OccupancyGridConstPtr &msg);

void printROSParams();

void loadROSParams();

void createROSComms();

// .............................................................................
// Global vars .................................................................
// .............................................................................

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>MoveBaseClient;
vector<int> occdata;
int costmapReceived = 0;
float costresolution;
int costwidth;
int costheight;
double min_pan_angle, max_pan_angle, min_tilt_angle, max_tilt_angle,
    sample_delay, tilt_angle;
int num_pan_sweeps, num_tilt_sweeps;
double sensing_range, offsetY_base_rmld, FoV;
int statusPTU, prevStatusPTU;
double timeOfScanning = 0;
bool btMode = false;
double min_robot_speed = 0.1;

// .............................................................................
//  ROS global vars ............................................................
// .............................................................................

NavigationUtilities nav_utils;
nav_msgs::GetPlan path;
nav_msgs::GetMap srv_map;
record_ros::String_cmd srv_rosbag;
geometry_msgs::Pose costorigin;
nav_msgs::OccupancyGrid costmap_grid;

// .............................................................................
//  ROS PARAMETERS .............................................................
// .............................................................................

std::string static_map_srv_name;
std::string make_plan_srv_name;
std::string move_base_goal_topic_name;
std::string move_base_srv_name;
std::string nav_grid_debug_topic_name;
std::string planning_grid_debug_topic_name;
std::string move_base_costmap_topic_name;
std::string move_base_local_costmap_topic_name;
std::string move_base_costmap_updates_topic_name;
std::string  marker_pub_topic_name;
std::string rosbag_srv_name;
double robot_radius;
bool bag_record;

// .............................................................................
// Ros services/subscribers/publishers
// .............................................................................

ros::ServiceClient map_service_client_;
ros::ServiceClient path_client;
ros::ServiceClient rosbag_client;
ros::Publisher moveBasePub;
ros::Subscriber costmap_sub;
ros::Subscriber costmap_update_sub;
ros::Publisher gridPub;
ros::Publisher planningPub;
ros::Publisher marker_pub;

// .............................................................................


// Input : ./mcdm_online_exploration_ros ./../Maps/map_RiccardoFreiburg_1m2.pgm
// 100 75 5 0 15 180 0.95 0.12
// resolution x y orientation range centralAngle precision threshold
int main(int argc, char **argv) {

  // input parameters
  double initFov;
  int initRange;
  double precision;
  double threshold;
  double resolution;

  // variables ......................................
  double path_len;
  bool path_srv_call;
  ros::Rate r(1);

  Pose start_pose;
  Pose target;
  Pose previous;
  Pose invertedInitial;
  Pose eastInitial;
  Pose westInitial;

  dummy::Map map;
  int gridToPathGridScale;
  std::pair<float, float> targetPos;
  long numConfiguration;
  vector<pair<string, list<Pose> >> graph2;
  bool backTracking;
  MCDMFunction function;
  long sensedCells;
  long newSensedCells;
  long totalFreeCells;
  int count;
  int countBT;
  double travelledDistance;
  int numOfTurning;
  double totalAngle;
  unordered_map<string, int> visitedCell;
  vector<string> history;
  NewRay ray;

  // amount of time the robot should do nothing for scanning the environment
  // ( final value expressed in second)
  unsigned int microseconds;
  list<Pose> unexploredFrontiers;
  list<Pose> tabuList;
  std::list<std::pair<float, float> > posToEsclude;
  list<Pose> nearCandidates;
  EvaluationRecords record;
  bool scan;
  double totalScanTime;
  int encodedKeyValue;

  // RFID
  double absTagX;  // m.
  double absTagY;  // m.
  double freq;     // Hertzs
  double txtPower; // dBs
  double rxPower;
  std::pair<int, int> relTagCoord;
  long i, j;
  long cell_i, cell_j;
  double targetX_meter, targetY_meter;
  bool success;


  // some param control ...
  if (argc < 6) {
    ROS_FATAL("Missing input arguments: Got (%d) and should be (%d) [Field of "
              "View, Sensing Range, Precision, Threshold, Resolution]",
              argc - 1, 6 - 1);
    return 1;
  }

  // input params
  initFov = atof(argv[1]) * M_PI / 180;
  initRange = atoi(argv[2]);
  precision = atof(argv[3]);
  threshold = atof(argv[4]);

  // derived from input params
  FoV = initFov;
  sensing_range = initRange;

  /* resolution = 0 -> full resolution
   * resolution = 1 -> 1mx1m
   * resolution = X -> X%(full resolution)
   *NOTE: LOWER RES VALUE, HIGHER REAL RESOLUTION*/
  resolution = atof(argv[5]);
  ROS_INFO("Parameters:");
  ROS_INFO("\t- Field of View  (initFov):   (%3.3f) rads", initFov );
  ROS_INFO("\t- Sensing Range  (InitRange): (%d) meters" , initRange );
  ROS_INFO("\t- Precision (precision):      (%3.3f)", precision);
  ROS_INFO("\t- Threshold (threshold):      (%3.3f)", threshold);
  ROS_INFO("\t- Resolution (resolution):      (%3.3f)", resolution);

  auto startMCDM = chrono::high_resolution_clock::now();
  ros::init(argc, argv, "mcdm_exploration_framework_node");

  //mfc Load params from ros
  loadROSParams();

  //mfc Load params from ros
  printROSParams();

  // create ROS connections/services
  createROSComms();

  // Start recording the bag
  if (bag_record){
    srv_rosbag.request.cmd = "record";
    if (rosbag_client.call(srv_rosbag)){
      ROS_DEBUG("Start recording the bag...");
      sleep(5);
    }else{
      ROS_ERROR("Error occurring while recording the bag. Exiting now!");
    }
  }

  // Wait for costamp
  while ( ros::ok() & (costmapReceived == 0) ) {
      ROS_INFO_STREAM_THROTTLE(60, "Waiting for costmap" << std::endl);
      ros::spinOnce();
      r.sleep();
    }

  // ...........................................................................
  // INITIALIZATIONS
  // ...........................................................................

  // Create maps from costmap
  map = dummy::Map(resolution, costmap_grid);
  ROS_DEBUG("Map created correctly");

  map.plotPathPlanningGridColor("/tmp/pathplanning_start.png");
  map.plotGridColor("/tmp/nav_start.png");

  //TODO check constructor ... 
  RFIDGridmap rfidGrid(costmap_grid, resolution, costresolution, false);
  ROS_DEBUG("RFID Map created correctly");

  gridToPathGridScale = map.getGridToPathGridScale();
  ROS_DEBUG("gridToPathGridScale: (%d)", gridToPathGridScale);

  // Get the initial pose in map frame
  start_pose = nav_utils.getCurrentPose(resolution, costresolution, &map, initFov, initRange);
  target = start_pose;
  previous = target;

  invertedInitial = nav_utils.createFromInitialPose(start_pose, M_PI, initRange, initFov);
  eastInitial = nav_utils.createFromInitialPose(start_pose, M_PI / 2, initRange, initFov);
  westInitial = nav_utils.createFromInitialPose(start_pose, 3 * M_PI / 2, initRange, initFov);

  numConfiguration = 1;
  backTracking = false;
  sensedCells = 0;
  newSensedCells = 0;
  totalFreeCells = map.getTotalFreeCells();
  count = 0;
  travelledDistance = 0;
  numOfTurning = 0;
  totalAngle = 0;
  ray.setGridToPathGridScale(gridToPathGridScale);


  // amount of time the robot should do nothing for scanning the environment
  // ( final value expressed in second)
  microseconds = 5 * 1000 * 1000;
  scan = true;
  totalScanTime = 0;
  encodedKeyValue = 0;

  // RFID
  absTagX = 0;  // m.
  absTagY = 0;  // m.
  freq = 0;     // Hertzs
  txtPower = 0; // dBs
  rxPower = 0;
  success = false;

  // ...........................................................................
  // main loop
  // ...........................................................................

  do {
    // If we are doing "forward" navigation towards cells never visited
    // before
    if (btMode == false) {

      //          if (count != 0)
      //          {
      //            move(target.getX(), target.getY(),
      //            target.getOrientation(), 10.0);
      //          }
      // At every iteration, the current pose of the robot is taken from the
      // TF-tree
      target = nav_utils.getCurrentPose(resolution, costresolution, &map, initFov, initRange);
      cout << "\n============================================" << endl;
      cout << "New iteration, position: " << target.getX() << "," << target.getY() <<
        "[ "<< newSensedCells << " sensed] - [" << totalFreeCells << " total]" <<
        "[ "<< 100 * float(newSensedCells)/float(totalFreeCells) << " %] - [" <<
        (chrono::duration<double, milli>(chrono::high_resolution_clock::now() - startMCDM).count() ) / 60000.0 << " min ]" << endl;

      map.getPathPlanningIndex(target.getX(), target.getY(), i, j);
      map.getPathPlanningPosition(targetX_meter, targetY_meter, i, j);
      map.getGridIndex(target.getX(), target.getY(), i, j);
      gridPub.publish(map.toMessageGrid());

      // Update starting point in the path
      path.request.start.header.frame_id = "map";
      path.request.start.pose.position.x = target.getX();
      path.request.start.pose.position.y = target.getY();
      path.request.start.pose.orientation.w = 1;

      float x = target.getX();
      float y = target.getY();
      float orientation = roundf(target.getOrientation() * 100) / 100;
      ; // cast orientation in [0, 360]
      int range = target.getRange();
      double FOV = target.getFOV();
      string actualPose = function.getEncodedKey(target, 0);
      map.setCurrentPose(target);
      string encoding = to_string(target.getX()) + to_string(target.getY());
      visitedCell.emplace(encoding, 0);
      // Get the sensing time required for scanning
      target.setScanAngles(
          map.getSensingTime(x, y, orientation, FOV, range));
      // Perform a scanning operation
      //          map.getGridIndex(x, y, cell_i, cell_j);
      //          newSensedCells = sensedCells + ray.performSensingOperation
      //          ( &map, x, y, orientation, FOV, range,
      //          target.getScanAngles().first,
      //          target.getScanAngles().second );
      newSensedCells =
          sensedCells +
          map.performSensingOperation(x, y, orientation, FOV, range,
                                      target.getScanAngles().first,
                                      target.getScanAngles().second);
      // Calculate the scanning angle
      double scanAngle =
          target.getScanAngles().second - target.getScanAngles().first;
      // Update the overall scanning time
      totalScanTime += nav_utils.calculateScanTime(scanAngle * 180 / M_PI);
      // Calculare the relative RFID tag position to the robot position
      //            relTagCoord = map.getRelativeTagCoord(absTagX, absTagY,
      //            target.getX(), target.getY());
      // Calculate the received power and phase
      //            double rxPower = received_power_friis(relTagCoord.first,
      //            relTagCoord.second, freq, txtPower);
      //            double phase = phaseDifference(relTagCoord.first,
      //            relTagCoord.second, freq);
      // Update the path planning and RFID map
      //          cout << endl << "[rfid_navigation.cpp]" <<
      //          map.getGridValue(target.getX() + 1, target.getY() + 1) <<
      //          endl;

      map.getPathPlanningIndex(x, y, cell_i, cell_j);
      //          range = range / costresolution;
      //          cout << "[rfid_navigation.cpp@main](x,y) = (" << x << ","
      //          << y << ")" << endl;
      map.updatePathPlanningGrid(x, y, range);
      //            rfidGrid.addEllipse(rxPower - SENSITIVITY,
      //            map.getNumGridCols() - target.getX(),  target.getY(),
      //            target.getOrientation(), -0.5, 7.0);
      // Search for new candidate position
      //          map.getGridIndex(x, y, cell_i, cell_j);

      gridPub.publish(map.toMessageGrid());
      planningPub.publish(map.toMessagePathPlanning());
      map.plotPathPlanningGridColor("/tmp/pathplanning_lastLoop.png");
      map.plotGridColor("/tmp/nav_lastLoop.png");
      map.findCandidatePositions(x, y, orientation, FOV, range);
      vector<pair<float, float>> candidatePosition = map.getCandidatePositions();

        map.emptyCandidatePositions();

      if (scan) {
        // NOTE: perform gas sensing------------
        //              offsetY_base_rmld = 0.904;
        //              tilt_angle = (atan(sensing_range /
        //              offsetY_base_rmld) * (180 / M_PI)) - 90;
        //              //tilt_angle = -10;
        //              num_pan_sweeps = 1;
        //              num_tilt_sweeps = 1;
        //              sample_delay = 0.1;
        //              //cout << "angolo finale:" << endl;
        //              max_pan_angle =
        //              getPtuAngle(target.getScanAngles().second,
        //              target.getOrientation());
        //              //cout << "angolo iniziale:" << endl;
        //              min_pan_angle =
        //              getPtuAngle(target.getScanAngles().first,
        //              target.getOrientation());
        //
        //              boost::thread mythread(scanning);
        //              mythread.join();
        //              min_pan_angle = 0;F
        //              max_pan_angle = 0;
        //-------------------------------------
      }
      // If the exploration just started
      if (count == 0) {
        // Calculate other three pose given the starting one
        string invertedPose = function.getEncodedKey(invertedInitial, 0);
        string eastPose = function.getEncodedKey(eastInitial, 0);
        string westPose = function.getEncodedKey(westInitial, 0);
        list<Pose> empty;
        std::pair<string, list<Pose> > pair1 =
            make_pair(invertedPose, empty);
        std::pair<string, list<Pose> > pair2 = make_pair(eastPose, empty);
        std::pair<string, list<Pose> > pair3 = make_pair(westPose, empty);
        // And add them (with empty candidates) to the graph structure
        graph2.push_back(pair1);
        graph2.push_back(pair2);
        graph2.push_back(pair3);
      }

      // If it's not the first step but we are in one of the initial
      // position (we come back here with backtracking)
      if (count != 0 &&
          (target.isEqual(invertedInitial) || target.isEqual(eastInitial) ||
           target.isEqual(westInitial))) {
        // If there are no more destination in the graph, terminates the
        // navigation
        if (graph2.size() == 0)
          break;
        graph2.pop_back();
        actualPose = function.getEncodedKey(target, 0);
        // Add to the graph the initial positions and the candidates from
        // there (calculated inside the function)
        nav_utils.pushInitialPositions(map, x, y, orientation, range, FOV, threshold,
                             actualPose, &graph2, &path_client);
      }

      // If there are no new candidate positions from the current pose of
      // the robot
      if (candidatePosition.size() == 0) {
        // Find candidates
        map.findCandidatePositions2(x, y, orientation, FOV, range);
        candidatePosition = map.getCandidatePositions();
        map.emptyCandidatePositions();

        cout << "No other candidate position" << endl;
        cout << "----- BACKTRACKING -----" << endl;

        // If the graph contains cells that can be explored
        if (graph2.size() > 1) {
          // Get the last position in the graph and then remove it
          string targetString = graph2.at(graph2.size() - 1).first;
          graph2.pop_back();
          //          EvaluationRecords record;
          target = record.getPoseFromEncoding(targetString);
          // Add it to the history as cell visited more than once
          history.push_back(function.getEncodedKey(target, 2));
          cout << "[BT]No significand position reachable. Come back to "
                  "previous position"
               << endl;
          cout << "       " << function.getEncodedKey(target, 0) << endl;
          count = count + 1;
          btMode = true;

          scan = false;
        }
        //...otherwise, if the graph does not contain cells that can be
        //explored
        // The navigation is finished!
        else {
          cout << "Num configuration: " << numConfiguration << endl;
          cout << "Travelled distance calculated during the algorithm: "
               << travelledDistance << endl;
          cout << "------------------ HISTORY -----------------" << endl;
          // Retrieve the cell visited only the first time
          list<Pose> tmp_history = nav_utils.cleanHistory(&history, &record);
          nav_utils.calculateDistance(tmp_history, &path_client, robot_radius);

          cout << "------------------ TABULIST -----------------" << endl;
          // Calculate the path connecting the cells in the tabulist, namely
          // the cells that are visited one time and couldn't be visite
          // again
          nav_utils.calculateDistance(tabuList, &path_client, robot_radius);

          // Normalise the travel distance in meter
          // NOTE: assuming that the robot is moving at 0.5m/s and the
          // resolution of the map is 0.5m per cell)
          if (resolution == 1.0) {
            travelledDistance = travelledDistance / 2;
          }
          nav_utils.printResult(newSensedCells, totalFreeCells, precision,
                      numConfiguration, travelledDistance, numOfTurning,
                      totalAngle, totalScanTime, resolution);
          auto endMCDM = chrono::high_resolution_clock::now();
          double totalTimeMCDM =
              chrono::duration<double, milli>(endMCDM - startMCDM).count();
          cout << "Total time for MCDM algorithm : " << totalTimeMCDM
               << "ms, " << totalTimeMCDM / 1000 << " s, "
               << totalTimeMCDM / 60000 << " m " << endl;
          cout << "Total time in empirical way : "
               << travelledDistance / 0.25 + timeOfScanning / 1000 << endl;

          // Stop recording the bag
          if (bag_record){
            srv_rosbag.request.cmd = "stop";
            if (rosbag_client.call(srv_rosbag)){
              cout << "Stop recording the bag..." << endl;
              sleep(5);
            }else{
              cout << "Error occurring while stopping recording the bag. Exiting now!" << endl;
            }
          }
          exit(0);
        }

        sensedCells = newSensedCells;

      }
      //... otherwise, if there are further candidate new position from the
      //current pose of the robot
      else {
        // need to convert from a <int,int pair> to a Pose with also
        // orientation,laser range and angle
        list<Pose> frontiers;
        // For every candidate positio, create 8 pose with a different
        // orientation each and consider them as frontiers
        vector<pair<float, float> >::iterator it = candidatePosition.begin();
        for (it; it != candidatePosition.end(); it++) {
          Pose p1 = Pose((*it).first, (*it).second, roundf(0 * 100) / 100,
                         range, FOV);
          // Pose p2 = Pose((*it).first, (*it).second,
          //                roundf(M_PI / 4 * 100) / 100, range, FOV);
          Pose p3 = Pose((*it).first, (*it).second,
                         roundf(M_PI / 2 * 100) / 100, range, FOV);
          // Pose p4 = Pose((*it).first, (*it).second,
          //                roundf(3 * M_PI / 4 * 100) / 100, range, FOV);
          Pose p5 = Pose((*it).first, (*it).second,
                         roundf(M_PI * 100) / 100, range, FOV);
          // Pose p6 = Pose((*it).first, (*it).second,
          //                roundf(5 * M_PI / 4 * 100) / 100, range, FOV);
          Pose p7 = Pose((*it).first, (*it).second,
                         roundf(3 * M_PI / 2 * 100) / 100, range, FOV);
          // Pose p8 = Pose((*it).first, (*it).second,
          //                roundf(7 * M_PI / 4 * 100) / 100, range, FOV);
          frontiers.push_back(p1);
          // frontiers.push_back(p2);
          frontiers.push_back(p3);
          // frontiers.push_back(p4);
          frontiers.push_back(p5);
          // frontiers.push_back(p6);
          frontiers.push_back(p7);
          // frontiers.push_back(p8);
        }
        unexploredFrontiers = frontiers;

        // Evaluate the frontiers and return a list of <frontier,
        // evaluation> pairs
        EvaluationRecords *record = function.evaluateFrontiers(frontiers, &map, threshold, &path_client);
        nearCandidates = record->getFrontiers();

        // Print the frontiers with the respective evaluation
        cout << "Number of frontiers identified: " << nearCandidates.size() << endl;
        unordered_map<string, double> evaluation = record->getEvaluations();
        // If there are candidate positions
        if (record->size() > 0) {
          // Set the previous pose equal to the current one (represented by
          // target)
          previous = target;
          // Select the new robot destination from the list of candidates
          std::pair<Pose, double> result = function.selectNewPose(record);
          target = result.first;

          cout << "Target selected: " << target.getX() << ", " << target.getY() << endl;
          target = nav_utils.selectFreePoseInLocalCostmap(target, &nearCandidates, &map, &function, threshold,
              &path_client, &posToEsclude, record, move_base_local_costmap_topic_name);
          targetPos = std::make_pair(target.getX(), target.getY());

          // If the selected destination does not appear among the cells
          // already visited
          if ((!nav_utils.containsPos(&posToEsclude, targetPos))) {
            //                                cout << "2" << endl;
            // Add it to the list of visited cells as first-view
            encodedKeyValue = 1;
            backTracking = false;

            success = nav_utils.showMarkerandNavigate(target, &marker_pub, &path, &path_client,
                                  &tabuList, &posToEsclude, min_robot_speed, robot_radius);
            if (success == true){
//                  cout << "[rfid_navigation.cpp@main] travelledDistance = " << travelledDistance << endl;
              nav_utils.updatePathMetrics(
                  &count, &target, &previous, actualPose, &nearCandidates,
                  &graph2, &map, &function, &tabuList, &posToEsclude,
                  &history, encodedKeyValue, &numConfiguration,
                  &totalAngle, &travelledDistance, &numOfTurning, scanAngle,
                  &path_client, backTracking, robot_radius);
//                  cout << "[rfid_navigation.cpp@main] travelledDistance = " << travelledDistance << endl;
            }

            scan = true;
          }
          // ...otherwise, if the selected cell has already been visited
          else {
            //                                cout << "3" << endl;
            // If the graph is empty, stop the navigation
            if (graph2.size() == 0)
              break;
            // If there still are more candidates to explore from the last
            // pose in the graph
            if (graph2.at(graph2.size() - 1).second.size() != 0) {
              cout << "[BT1 - Tabulist]There are visible cells but the "
                      "selected one is already "
                      "(or cannot be) explored!Come back to second best "
                      "position from the previous position"
                   << endl;
              // Remove the current position from possible candidates
              nearCandidates = graph2.at(graph2.size() - 1).second;
              nav_utils.cleanPossibleDestination2(&nearCandidates, target);
              // Get the list of new candidate position with associated
              // evaluation
              record = function.evaluateFrontiers(nearCandidates, &map,
                                                  threshold, &path_client);
              // If there are candidate positions
              cout << "PoseToEsclude:" << endl;
              for (auto iter = posToEsclude.begin(); iter != posToEsclude.end(); iter++) {
                  cout << " " << iter->first << "," << iter->second << endl;
              }
              while (1) {
                if (record->size() != 0) {
                  // Select the new pose of the robot
                  std::pair<Pose, double> result = function.selectNewPose(record);
                  target = result.first;
                  targetPos = make_pair(target.getX(), target.getY());
                  //                      if (!contains(tabuList, target)) {
                  if (!nav_utils.containsPos(&posToEsclude, targetPos)) {
                    // If the new selected position is not in the Tabulist

                    encodedKeyValue = 1;
                    scan = false;
                    // Set that we are now in backtracking
                    cout << "[BT1] Break the while" << endl;
                    break; // the while loop
                  } else {
                    // Remove the current position from possible candidates
                    nav_utils.cleanPossibleDestination2(&nearCandidates, target);
                    // Get the list of new candidate position with
                    // associated evaluation
                    record = function.evaluateFrontiers(nearCandidates, &map, threshold, &path_client);
                  }
                }
                // If there are no more candidate position from the last
                // position in the graph
                else {
                  cout << "[BT2 - New]There are visible cells but the selected one is already "
                          "explored! Come back to best frontier from the two positions back in the graph. Start selecting the new record"
                       << endl;
                  // Remove the last element (cell and associated candidate from
                  // there) from the graph
                  if (graph2.size() == 1) break;
                  graph2.pop_back();
                  // Select the new record from two position back in the graph
                  nearCandidates = graph2.at(graph2.size() - 1).second;
                  record = function.evaluateFrontiers(nearCandidates, &map,
                                                      threshold, &path_client);
                }
              }
              cout << "[BT1-2]Target: " << target.getX() << ", " << target.getY() << endl;
              backTracking = true;
              previous = nav_utils.getCurrentPose(resolution, costresolution, &map, initFov, initRange);
              success = nav_utils.showMarkerandNavigate(target, &marker_pub, &path,
                                    &path_client, &tabuList, &posToEsclude, min_robot_speed, robot_radius);
              if (success == true)
              {
//                    cout << "[rfid_navigation.cpp@main] travelledDistance = " << travelledDistance << endl;
                nav_utils.updatePathMetrics(
                    &count, &target, &previous, actualPose, &nearCandidates,
                    &graph2, &map, &function, &tabuList, &posToEsclude,
                    &history, encodedKeyValue, &numConfiguration,
                    &totalAngle, &travelledDistance, &numOfTurning, scanAngle,
                    &path_client, backTracking, robot_radius);
//                    cout << "[rfid_navigation.cpp@main] travelledDistance = " << travelledDistance << endl;
              }
              scan = true;
            }
            // ... if the graph still does not present anymore candidate
            // positions for its last pose
            else {
              cout << "[BT2 - Tabulist]There are visible cells but the "
                      "selected one is already "
                      "explored! Come back to two positions ago"
                   << endl;
              // Remove the last element (cell and associated candidate from
              // there) from the graph
              if (graph2.size() == 1) break;
              graph2.pop_back();
              // Select as new target, the new last element of the graph
              string targetString = graph2.at(graph2.size() - 1).first;
              nearCandidates = graph2.at(graph2.size() - 1).second;
              target = record->getPoseFromEncoding(targetString);
              // Save it history as cell visited more than once
              history.push_back(function.getEncodedKey(target, 2));

              count = count + 1;
              scan = false;
              btMode = true;
            }
          }
        }
        // ... otherwise, if there are no candidate positions
        else {
          // If the graph is empty, stop the navigation
          if (graph2.size() == 0)  break;

          cout << "[BT3] There are no visible cells so come back to "
                  "previous position in the graph"
                  " structure:"
               << endl;
          // Select as new target the last one in the graph structure
          string targetString = graph2.at(graph2.size() - 1).first;
          nearCandidates = graph2.at(graph2.size() - 1).second;
          // Remove it from the graph
          graph2.pop_back();
          target = record->getPoseFromEncoding(targetString);
          cout << "--> " << function.getEncodedKey(target, 0) << endl;
          cout << "Previous: " << function.getEncodedKey(previous, 2)
               << endl;
          // Check if the selected cell in the graph is the previous robot
          // position
          if (!target.isEqual(previous)) {
            // if it's not, set the old position as the current one
            previous = target; // TODO: WHY?

            // Save the new target in the history as cell visited more than
            // once
            history.push_back(function.getEncodedKey(target, 2));
            count = count + 1;
            btMode = true;
            scan = true;
          }
          // If the selected cell is the old robot position
          else {
            // If there are no more cells in the graph, just finish the
            // navigation
            if (graph2.size() == 0)
              break;
            // Select the last position in the graph
            string targetString = graph2.at(graph2.size() - 1).first;
            nearCandidates = graph2.at(graph2.size() - 1).second;
            // and remove it from the graph
            graph2.pop_back();
            target = record->getPoseFromEncoding(targetString);
            // Set the previous pose as the current one
            previous = target;
            cout << "[BT5]There are no visible cells so come back to "
                    "previous position"
                 << endl;
            cout << "[BT5]Cell already explored!Come back to previous "
                    "position"
                 << endl;
            // Add it in history as cell visited more than once
            history.push_back(function.getEncodedKey(target, 2));
            count = count + 1;
            btMode = true;
          }
        }
        // NOTE: not requested for testing purpose
        // usleep(microseconds);
        sensedCells = newSensedCells;
        frontiers.clear();
        candidatePosition.clear();
        delete record;
      }

    }
    // ... otherwise, if we are doing backtracking
    else {
      cout << "-------------- BTMODE --------------" << endl;
//          cout << "Previous: " << previous.getX() << ", " << previous.getY() << endl;
//          cout << "Target: " << target.getX() << ", " << target.getY() << endl;
      float x = target.getX();
      float y = target.getY();
      float orientation = roundf(target.getOrientation() * 100) / 100;
      int range = target.getRange();
      double FOV = target.getFOV();
      string actualPose = function.getEncodedKey(target, 0);
      map.setCurrentPose(target);
      // NOTE; calculate path and turnings between actual position and goal
      string encoding = to_string(target.getX()) + to_string(target.getY());
      visitedCell.emplace(encoding, 0);
      //                    // Set the previous cell to be the same of the
      //                    current one
      //                    previous = target;

      // Calculate how much time it takes to scan the current area
      target.setScanAngles(map.getSensingTime(x, y, orientation, FOV, range));
      // Get the scanning angle
      double scanAngle = target.getScanAngles().second - target.getScanAngles().first;
      // Update the overall scanned angle
      totalAngle += scanAngle;
      // ...and the overall scan time
      totalScanTime += nav_utils.calculateScanTime(scanAngle * 180 / M_PI);
      // Calculate the relative coordinate to the robot of the RFID tag
      //            relTagCoord = map.getRelativeTagCoord(absTagX, absTagY,
      //            target.getX(), target.getY());
      // Calculate received power and phase
      //            double rxPower = received_power_friis(relTagCoord.first,
      //            relTagCoord.second, freq, txtPower);
      //            double phase = phaseDifference(relTagCoord.first,
      //            relTagCoord.second, freq);
      //          cout << "[rfid_navigation.cpp@main](x,y) = (" << x << ","
      //          << y << ")" << endl;
//          map.updatePathPlanningGrid(x, y, range);
      //            rfidGrid.addEllipse(rxPower - SENSITIVITY,
      //            map.getNumGridCols() - target.getX(), target.getY(),
      //            target.getOrientation(), -0.5, 7.0);
      // Remove the current pose from the list of possible candidate cells
      nav_utils.cleanPossibleDestination2(&nearCandidates, target);
//          cout << "Cleaned" << endl;
      // Get the list of the candidate cells with their evaluation
      EvaluationRecords *record = function.evaluateFrontiers(nearCandidates, &map, threshold, &path_client);
//          cout << "Record obtained, size is " << record->size() << endl;

      // If there are candidate cells
      if (record->size() > 0) {
        // Find the new destination
        std::pair<Pose, double> result = function.selectNewPose(record);
        target = result.first;
        targetPos = make_pair(target.getX(), target.getY());

        // If this cells has not been visited before
        //            if ( ! contains ( tabuList,target ) )
        if ((!nav_utils.containsPos(&posToEsclude, targetPos))) {

          // Add it to the list of visited cells as first-view
          encodedKeyValue = 1;
          backTracking = true;
          // Update the current pose
          previous = nav_utils.getCurrentPose(resolution, costresolution, &map, initFov, initRange);
          success = nav_utils.showMarkerandNavigate(target, &marker_pub, &path, &path_client,
                                &tabuList, &posToEsclude, min_robot_speed, robot_radius);
          if (success == true){
//                cout << "[rfid_navigation.cpp@main] travelledDistance = " << travelledDistance << endl;
            nav_utils.updatePathMetrics(
                &count, &target, &previous, actualPose, &nearCandidates,
                &graph2, &map, &function, &tabuList, &posToEsclude,
                &history, encodedKeyValue,  &numConfiguration,
                &totalAngle, &travelledDistance, &numOfTurning, scanAngle,
                &path_client, backTracking, robot_radius);
//                cout << "[rfid_navigation.cpp@main] travelledDistance = " << travelledDistance << endl;
          }
          // Leave the backtracking branch
          btMode = false;
          nearCandidates.clear();
          cout << "[BT-MODE4] Go back to the best frontiers from the "
                  "previous positions in the graph"
               << endl;
          cout << "       " << function.getEncodedKey(target, 0) << endl;
        }
        // ... otherwise, if the cells has already been visisted
        else {
          // If there are other candidates
          if (nearCandidates.size() != 0) {
            cout << "[BT-MODE1]Already visited, but there are other "
                    "candidates"
                 << endl;

            // Remove the destination from the candidate list
            nav_utils.cleanPossibleDestination2(&nearCandidates, target);
            // Get the candidates with their evaluation
            EvaluationRecords *record = function.evaluateFrontiers(
                nearCandidates, &map, threshold, &path_client);
            // Select the new destination
            std::pair<Pose, double> result = function.selectNewPose(record);
            target = result.first;

            // Add it to the list of visited cells as first-view
            encodedKeyValue = 1;
            backTracking = true;
            nav_utils.updatePathMetrics(
                &count, &target, &previous, actualPose, &nearCandidates,
                &graph2, &map, &function, &tabuList, &posToEsclude,
                &history, encodedKeyValue,  &numConfiguration,
                &totalAngle, &travelledDistance, &numOfTurning, scanAngle,
                &path_client, backTracking, robot_radius);
          }
          // ...otherwise, if there are no more candidates
          else {
            if (graph2.size() == 0) break;
            // Select as target the last element in the graph
            string targetString = graph2.at(graph2.size() - 1).first;
            nearCandidates = graph2.at(graph2.size() - 1).second;
            // And remove from the graph
            graph2.pop_back();
            target = record->getPoseFromEncoding(targetString);
            // Add it to the history of cell as already more than once
            encodedKeyValue = 2;
            // Leave backtracking
            btMode = true;
            // Clear candidate list
            nearCandidates.clear();
            cout << "[BT-MODE2] Go back to previous positions in the graph"
                 << endl;
            cout << "       " << targetString << endl;
          }
        }
      }
      // ... if there are not candidate cells
      else {
        // Select as new pose, the last cell in the graph
        while (1) {
          if (graph2.size() != 0) { // If there are still position on which
                                    // doing backtracking
            if ((graph2.at(graph2.size() - 1).second).size() >
                0) // If there are still frontiers from the current position
            {
              string targetString = graph2.at(graph2.size() - 1).first;
              nearCandidates = graph2.at(graph2.size() - 1).second;
              target = record->getPoseFromEncoding(targetString);
              // and the remove it form the graph
              graph2.pop_back();
              // Leave backtracking
              btMode = true;
              cout << "[BT-MODE3] There are no candidate frontiers in the "
                      "record. Go back to previous positions in the graph"
                   << endl;
              cout << "       " << targetString << endl;
              break;
            } else {
              cout << "[BT-MODE3] No more frontiers with the associate "
                      "position in the graph"
                   << endl;
              graph2.pop_back();
            }
          } else {
            cout << "[BT-MODE3] Graph2 is empty. Navigation is finished"
                 << endl;
            break;
          }
        }
        if (graph2.size() == 0) break;
      }
      delete record;
    }
  }
  // Perform exploration until a certain coverage is achieved
  while (sensedCells < precision * totalFreeCells);

  ROS_DEBUG("[rfid_navigation.cpp@main] End condition reached ...");

  // ...........................................................................
  // aftermatch
  // ...........................................................................

  // Plotting utilities
  map.drawVisitedCells();
  map.printVisitedCells(history);
  map.drawRFIDScan();
  map.drawRFIDGridScan(rfidGrid);
  rfidGrid.saveAs(("/tmp/rfid_result_gridmap.pgm"));

  ROS_INFO("Num configuration: (%lu)", numConfiguration );
  ROS_INFO("Travelled distance calculated during the algorithm: (%3.3f)", travelledDistance);
  ROS_INFO(" ------------------ HISTORY -----------------");

  // Calculate which cells have been visited only once
  list<Pose> tmp_history = nav_utils.cleanHistory(&history, &record);
  nav_utils.calculateDistance(tmp_history, &path_client, robot_radius);

  ROS_INFO("------------------ TABULIST -----------------");
  nav_utils.calculateDistance(tabuList, &path_client, robot_radius);

  // Trasform distance in meters
  // Corridor map has a resolution of 0.5 meter per cell
  if (resolution == 1.0) {
    travelledDistance = travelledDistance / 2;
  }

  nav_utils.printResult(newSensedCells, totalFreeCells, precision, numConfiguration,
              travelledDistance, numOfTurning, totalAngle, totalScanTime, resolution);

  // Find the tag
  std::pair<int,int> tag = map.findTag();
  ROS_INFO("RFID pose: [%d, %d]", tag.second, tag.first);
  tag = map.findTagfromGridMap(rfidGrid);
  ROS_INFO("[Grid]RFID pose: [%d, %d]", tag.second, tag.first);

  ROS_INFO("-----------------------------------------------------------------");
  auto endMCDM = chrono::high_resolution_clock::now();

  double totalTimeMCDM = chrono::duration<double, milli>(endMCDM - startMCDM).count();
  ROS_INFO("Total time for MCDM algorithm : (%3.3f) msec., (%3.3f) sec., (%3.3f) min.",totalTimeMCDM , totalTimeMCDM / 1000 , totalTimeMCDM / 60000);
  ROS_INFO("Spinning at the end");

  // Stop recording the bag
  if (bag_record){
    srv_rosbag.request.cmd = "stop";
    if (rosbag_client.call(srv_rosbag)){
      ROS_DEBUG("Stop recording the bag...");
      sleep(5);
    }else{
      ROS_ERROR("Error occurring while stop recording the bag. Exiting now!");
    }
  }

  ROS_DEBUG("[rfid_navigation.cpp@main] End. Bye!");
  ros::shutdown();

} // end main


void grid_callback(const nav_msgs::OccupancyGridConstPtr &msg) {
  if (costmapReceived == 0) {
    costmap_grid = *msg;
    costresolution = msg->info.resolution;
    costwidth = msg->info.width;
    costheight = msg->info.height;
    costorigin = msg->info.origin;
    for (int i = 0; i < msg.get()->data.size(); ++i) {
      occdata.push_back(msg->data.at(i));
    }

    ROS_DEBUG("Costmap received:");
    ROS_DEBUG("\t - size of occdata: (%lu) ", occdata.size() );
    ROS_DEBUG("\t - size of message data: (%lu) ", msg->data.size()  );
    ROS_DEBUG("\t - height: (%d) ", msg->info.height  );
    ROS_DEBUG("\t - width: (%d) ", msg->info.width );
    ROS_DEBUG("\t - resolution (costresolution): (%3.3f) ", msg->info.resolution);
    costmapReceived = 1;
  }
}

void update_callback(const map_msgs::OccupancyGridUpdateConstPtr &msg) {
  // NOTE: everything is commented because we don't want update the costmap
  // since the environment is
  // assumed static

  // std::cout << "CALLBACK SECOND" << std::endl;

  /*int index = 0;
      for(int y=msg->y; y< msg->y+msg->height; y++) {
              for(int x=msg->x; x< msg->x+msg->width; x++) {
                      costmap_grid.data[ getIndex(x,y) ] = msg->data[ index++ ];
              }
      }*/
}


void loadROSParams(){

  ros::NodeHandle private_node_handle("~");

  // LOAD ROS PARAMETERS ....................................
  private_node_handle.param("static_map_srv_name", static_map_srv_name, std::string("static_map"));
  private_node_handle.param("/move_base/global_costmap/robot_radius", robot_radius, 0.25);
  private_node_handle.param("make_plan_srv_name", make_plan_srv_name, std::string("/move_base/make_plan"));
  private_node_handle.param("move_base_goal_topic_name", move_base_goal_topic_name, std::string("move_base_simple/goal"));
  private_node_handle.param("move_base_srv_name", move_base_srv_name, std::string("move_base"));
  private_node_handle.param("nav_grid_debug_topic_name", nav_grid_debug_topic_name, std::string("nav_grid_debug"));
  private_node_handle.param("planning_grid_debug_topic_name", planning_grid_debug_topic_name, std::string("planning_grid_debug"));
  private_node_handle.param("move_base_costmap_topic_name", move_base_costmap_topic_name, std::string("move_base/global_costmap/costmap"));
  private_node_handle.param("move_base_costmap_updates_topic_name", move_base_costmap_updates_topic_name, std::string("move_base/global_costmap/costmap_updates"));
  private_node_handle.param("move_base_local_costmap_topic_name", move_base_local_costmap_topic_name, std::string("/move_base/local_costmap/costmap"));
  private_node_handle.param("marker_pub_topic_name", marker_pub_topic_name, std::string("goal_pt"));
  private_node_handle.param("bag_record", bag_record, false);
  private_node_handle.param("rosbag_srv_name", rosbag_srv_name, std::string("/record/cmd"));


}

void printROSParams(){
  ROS_DEBUG("/////////////////////////////////////////////////////////////////////////");
  ROS_DEBUG("[pure_navigation@printROSParams] Using the following ros params:");
  ROS_DEBUG("   - robot_radius [%3.3f]",  robot_radius);
  ROS_DEBUG("   - static_map_srv_name [%s]", static_map_srv_name.c_str());
  ROS_DEBUG("   - make_plan_srv_name [%s]", make_plan_srv_name.c_str());
  ROS_DEBUG("   - move_base_goal_topic_name [%s]", move_base_goal_topic_name.c_str());
  ROS_DEBUG("   - move_base_srv_name [%s]", move_base_srv_name.c_str());
  ROS_DEBUG("   - nav_grid_debug_topic_name [%s]", nav_grid_debug_topic_name.c_str());
  ROS_DEBUG("   - planning_grid_debug_topic_name [%s]", planning_grid_debug_topic_name.c_str());
  ROS_DEBUG("   - move_base_costmap_topic_name [%s]", move_base_costmap_topic_name.c_str());
  ROS_DEBUG("   - move_base_costmap_updates_topic_name [%s]", move_base_costmap_updates_topic_name.c_str());
  ROS_DEBUG("   - marker_pub_topic_name [%s]", marker_pub_topic_name.c_str());
  ROS_DEBUG("   - bag_record [%s]", bag_record ? "true" : "false" );
  ROS_DEBUG("   - rosbag_srv_name [%s]", rosbag_srv_name.c_str());
  ROS_DEBUG("/////////////////////////////////////////////////////////////////////////");

}

void createROSComms(){

  ros::NodeHandle nh;
  ros::Rate r(20);
  bool disConnected = true;

  // create service clients
  map_service_client_ = nh.serviceClient<nav_msgs::GetMap>(static_map_srv_name);
  path_client =   nh.serviceClient<nav_msgs::GetPlan>(make_plan_srv_name, true);
  rosbag_client = nh.serviceClient<record_ros::String_cmd>(rosbag_srv_name);

  // create publishers
  moveBasePub =   nh.advertise<geometry_msgs::PoseStamped>(move_base_goal_topic_name, 1000);
  gridPub = nh.advertise<grid_map_msgs::GridMap>(nav_grid_debug_topic_name, 1, true);
  planningPub = nh.advertise<grid_map_msgs::GridMap>(planning_grid_debug_topic_name, 1, true);
  marker_pub =  nh.advertise<geometry_msgs::PointStamped>(marker_pub_topic_name, 10);

  while (disConnected) {
    cout << "[pure_navigation@createROSComms] Waiting for static_map service to respond..." << endl;
    if (map_service_client_.call(srv_map)) {
      costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>(
           move_base_costmap_topic_name, 100, grid_callback);
      costmap_update_sub = nh.subscribe<map_msgs::OccupancyGridUpdate>(
           move_base_costmap_updates_topic_name, 10, update_callback);
      disConnected = false;
    } else {
      r.sleep();
    }
  }


}
