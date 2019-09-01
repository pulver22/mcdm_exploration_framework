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
// #include "RFIDGridmap.h"
#include "movebasegoal.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
#include <std_msgs/Float32.h>
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

// ROS varies
bool move(float x, float y, float orientation, float time_travel,
          list<Pose> *tabuList,
          std::list<std::pair<float, float> > *posToEsclude);
void update_callback(const map_msgs::OccupancyGridUpdateConstPtr &msg);
void grid_callback(const nav_msgs::OccupancyGridConstPtr &msg);
void printROSParams();
void loadROSParams();
void createROSComms();
Pose getRandomFrontier(list<Pose> nearCandidates);
void tag_coverage_callback(const std_msgs::Float32 msg);

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>MoveBaseClient;
vector<int> occdata;
int costmapReceived = 0;
float costresolution;
int costwidth;
int costheight;
geometry_msgs::Pose costorigin;
nav_msgs::OccupancyGrid costmap_grid;
double min_pan_angle, max_pan_angle, min_tilt_angle, max_tilt_angle,
    sample_delay, tilt_angle;
int num_pan_sweeps, num_tilt_sweeps;
double sensing_range, offsetY_base_rmld, FoV;
int statusPTU, prevStatusPTU;
double timeOfScanning = 0;
bool btMode = false;
double min_robot_speed = 0.1;
nav_msgs::GetPlan path;
float tag_coverage_percentage = 0.0;

//  ROS PARAMETERS ....................................
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
NavigationUtilities nav_utils;

// Ros services/subscribers/publishers
ros::ServiceClient map_service_client_;
ros::ServiceClient path_client;
ros::ServiceClient rosbag_client;
nav_msgs::GetMap srv_map;
ros::Publisher moveBasePub;
ros::Subscriber costmap_sub;
ros::Subscriber costmap_update_sub;
ros::Subscriber tag_coverage_sub;
ros::Publisher gridPub;
ros::Publisher planningPub;
ros::Publisher marker_pub;
record_ros::String_cmd srv_rosbag;

// Input : ./mcdm_online_exploration_ros ./../Maps/map_RiccardoFreiburg_1m2.pgm
// 100 75 5 0 15 180 0.95 0.12
// resolution x y orientation range centralAngle precision threshold
int main(int argc, char **argv) {

  // mfc ...........................
  // some param control ...
  if (argc < 6) {
    ROS_FATAL("Missing input arguments: Got (%d) and should be (%d) [Field of "
              "View, Sensing Range, Precision, Threshold,Resolution]",
              argc - 1, 6 - 1);
    return 1;
  } else {
    printf("\nParameters:\n- Field of View (%3.3f)\n- Sensing Range (%d)\n- "
             "Precision (%3.3f)\n- Threshold (%3.3f)\n- Resolution: (%3.3f)",
             atof(argv[1]), atoi(argv[2]), atof(argv[3]), atof(argv[4]),
             atof(argv[5]));
  }

  //   sets console output to debug mode...
  //  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //  ros::console::levels::Debug) )
  //  {
  //   ros::console::notifyLoggerLevelsChanged();
  //  }
  // mfc ...........................
  ros::init(argc, argv, "mcdm_exploration_framework_node");

  //mfc Load params from ros
  loadROSParams();

  //mfc Load params from ros
  printROSParams();

  // create ROS connections/services
  createROSComms();
  double path_len;
  bool path_srv_call;
  ros::Rate r(1);

  // Start recording the bag
  srv_rosbag.request.cmd = "record";
  if (rosbag_client.call(srv_rosbag)){
    cout << "Start recording the bag..." << endl;
    sleep(5);
  }else{
    cout << "Error occurring while recording the bag. Exiting now!" << endl;
//    exit(0);
  }


  while (ros::ok()) {

    if (costmapReceived == 0) {
      ROS_INFO_STREAM_THROTTLE(60, "waiting for costmap" << std::endl);
      // cout << "Waiting for costmap" << endl;
    }
    if (costmapReceived == 1) {
      double initFov = atof(argv[1]);
      initFov = initFov * M_PI / 180;
      FoV = initFov;
      int initRange = atoi(argv[2]);
      sensing_range = initRange;
      double precision = atof(argv[3]);
      double threshold = atof(argv[4]);

      /* resolution = 0 -> full resolution
       * resolution = 1 -> 1mx1m
       * resolution = X -> X%(full resolution)
       *NOTE: LOWER RES VALUE, HIGHER REAL RESOLUTION*/
      double resolution = atof(argv[5]);
      double w_info_gain = 0.33;
      double w_travel_distance = 0.33;
      double w_sensing_time = 0.33;
      std::string out_log = "/home/pulver/Desktop/MCDM/random_frontier/gazebo_inb3123_v2.csv";
      std::string coverage_log = "/home/pulver/Desktop/MCDM/random_frontier/gazebo_inb3123_coverage_v2.csv";
      cout << "Config: " << endl;
      cout << "   InitFov: " << initFov << endl;
      cout << "   InitRange: " << initRange << endl;
      cout << "   precision: " << precision << endl;
      cout << "   threshold: " << threshold << endl;

      cout << "   Resolution: " << resolution
           << "\n   Costresolution: " << costresolution << endl;

      // dummy::Map map = dummy::Map(costresolution, costresolution, costwidth,
      // costheight, occdata, costorigin);
      dummy::Map map = dummy::Map(resolution, costmap_grid);
      ROS_DEBUG("Map created correctly");

      map.plotPathPlanningGridColor("/tmp/pathplanning_start.png");

      map.plotGridColor("/tmp/nav_start.png");

      //        RFIDGridmap myGrid(argv[1], resolution, costresolution, false);
      //        cout << "RFIDgrid created correctly" << endl;
      int gridToPathGridScale = map.getGridToPathGridScale();
      cout << "gridToPathGridScale: " << gridToPathGridScale << endl;
      ROS_DEBUG("[pure_navigation.cpp@main] grid To Path Grid Scale obtained");

      /*NOTE: Transform between map and image, to be enabled if not present in
      the launch file
      //tf::Transform tranMapToImage;
      //tranMapToImage.setOrigin(tf::Vector3(0, 30, 0.0));
      //tf::Vector3 vecImageToMap = tf::Vector3(0, 30,0.0);
      */

      // Get the initial pose in map frame
      Pose start_pose =
          nav_utils.getCurrentPose(resolution, costresolution, &map, initFov, initRange);
      Pose target = start_pose;
      Pose previous = target;
      Pose invertedInitial = nav_utils.createFromInitialPose(start_pose, M_PI, initRange, initFov);
      Pose eastInitial = nav_utils.createFromInitialPose(start_pose, M_PI / 2, initRange, initFov);
      Pose westInitial = nav_utils.createFromInitialPose(start_pose, 3 * M_PI / 2, initRange, initFov);
      std::pair<float, float> targetPos;
      long numConfiguration = 1;
      vector<pair<string, list<Pose> >> graph2;
      bool backTracking = false;
      NewRay ray;
      ray.setGridToPathGridScale(gridToPathGridScale);
      MCDMFunction function(w_info_gain, w_travel_distance, w_sensing_time);
      long sensedCells = 0;
      long newSensedCells = 0;
      long totalFreeCells = map.getTotalFreeCells();
      int count = 0;
      int countBT;
      double travelledDistance = 0;
      int numOfTurning = 0;
      double totalAngle = 0;
      unordered_map<string, int> visitedCell;
      vector<string> history;
      //      history.push_back(function.getEncodedKey(target, 1));
      // amount of time the robot should do nothing for scanning the environment
      // ( final value expressed in second)
      unsigned int microseconds = 5 * 1000 * 1000;
      // cout << "total free cells in the main: " << totalFreeCells << endl;
      list<Pose> tabuList;
      std::list<std::pair<float, float> > posToEsclude;
      list<Pose> nearCandidates;
      EvaluationRecords record;
      bool scan = true;
      double totalScanTime = 0;
      int encodedKeyValue = 0;

      // RFID
      double absTagX = 0;  // std::stod(argv[12]); // m.
      double absTagY = 0;  // std::stod(argv[11]); // m.
      double freq = 0;     // std::stod(argv[13]); // Hertzs
      double txtPower = 0; // std::stod(argv[14]); // dBs
      double rxPower = 0;
      std::pair<int, int> relTagCoord;
      long i, j;
      long cell_i, cell_j;
      double targetX_meter, targetY_meter;
      bool success = false;

      float x;
      float y;
      float orientation;
      int range;
      double FOV;

      auto startMCDM = ros::Time::now().toSec();
      string content;

      do {
//        cout << "Graph size: " << graph2.size() << endl;
//        if (graph2.size() == 0 and count != 0)
//          break;
//        for (auto it = graph2.begin(); it != graph2.end(); it++) {
//          cout << " " << it->first << endl;
//        }

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
            (ros::Time::now().toSec() - startMCDM ) / 60.0 << " min ]" << endl;

          content = to_string(numConfiguration) + "," + to_string(100 * float(newSensedCells)/float(totalFreeCells)) + "," + to_string(tag_coverage_percentage) + "\n" ;
          nav_utils.saveCoverage(coverage_log, content, true );
          cout << "  ==> Saving the coverage log ..." << endl;

          map.getPathPlanningIndex(target.getX(), target.getY(), i, j);
          map.getPathPlanningPosition(targetX_meter, targetY_meter, i, j);
          map.getGridIndex(target.getX(), target.getY(), i, j);
          gridPub.publish(map.toMessageGrid());

          // Update starting point in the path
          path.request.start.header.frame_id = "map";
          path.request.start.pose.position.x = target.getX();
          path.request.start.pose.position.y = target.getY();
          path.request.start.pose.orientation.w = 1;

          x = target.getX();
          y = target.getY();
          orientation = roundf(target.getOrientation() * 100) / 100;
          ; // cast orientation in [0, 360]
          range = target.getRange();
          FOV = target.getFOV();
          string actualPose = function.getEncodedKey(target, 0);
          map.setCurrentPose(target);
          string encoding = to_string(target.getX()) + to_string(target.getY());
          visitedCell.emplace(encoding, 0);
          // Get the sensing time required for scanning
          target.setScanAngles(
              map.getSensingTime(x, y, orientation, FOV, range));
          newSensedCells = sensedCells + map.performSensingOperation(x, y, orientation, FOV, range,
                                          target.getScanAngles().first,
                                          target.getScanAngles().second);
          // Calculate the scanning angle
          double scanAngle =
              target.getScanAngles().second - target.getScanAngles().first;
          // Update the overall scanning time
          totalScanTime += nav_utils.calculateScanTime(scanAngle * 180 / M_PI);


          map.getPathPlanningIndex(x, y, cell_i, cell_j);
          map.updatePathPlanningGrid(x, y, range);

          gridPub.publish(map.toMessageGrid());
          planningPub.publish(map.toMessagePathPlanning());
          map.plotPathPlanningGridColor("/tmp/pathplanning_lastLoop.png");
          map.plotGridColor("/tmp/nav_lastLoop.png");
          map.findCandidatePositions(x, y, orientation, FOV, range);
          vector<pair<float, float>> candidatePosition = map.getCandidatePositions();
          map.emptyCandidatePositions();
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
                                 actualPose, &graph2, &path_client, &function);
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
                                    totalAngle, totalScanTime, resolution,
                                    w_info_gain, w_travel_distance, w_sensing_time, out_log);
              auto endMCDM = ros::Time::now().toSec();
              double totalTimeMCDM = endMCDM - startMCDM;
              cout << "Total time for MCDM algorithm : " << totalTimeMCDM
                   << "s, " << totalTimeMCDM / 60 << " m " << endl;
              cout << "Total time in empirical way : "
                   << travelledDistance / 0.25 + timeOfScanning / 1000 << endl;

              // Stop recording the bag
              srv_rosbag.request.cmd = "stop";
              if (rosbag_client.call(srv_rosbag)){
                cout << "Stop recording the bag..." << endl;
                sleep(5);
              }else{
                cout << "Error occurring while stopping recording the bag. Exiting now!" << endl;
              }
              ros::shutdown();
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


            // NOTE: This may not be needed because  we are in an unexplored area
            cout << "Number of frontiers identified: " << frontiers.size() << endl;
            nav_utils.cleanPossibleDestination2(&frontiers, target);
            nav_utils.cleanDestinationFromTabulist(&frontiers, &posToEsclude);
            // Print the frontiers with the respective evaluation
            cout << "Number of frontiers identified: " << frontiers.size() << endl;
            // If there are candidate positions
            // Evaluate the frontiers and return a list of <frontier,
            // evaluation> pairs
            EvaluationRecords *record = function.evaluateFrontiers(&frontiers, &map, threshold, &path_client);
            nearCandidates = record->getFrontiers();
            if (record->size() > 0) {
                        // EvaluationRecords *record;
                        // nearCandidates = frontiers;
                        // if (nearCandidates.size() > 0) {
              // Set the previous pose equal to the current one (represented by
              // target)
              previous = target;
              // Select the new RANDOM robot destination from the list of candidates
              target = getRandomFrontier(nearCandidates);
              cout << "Target selected: " << target.getX() << ", " << target.getY() << endl;
              target = nav_utils.selectFreePoseInLocalCostmap(target, &nearCandidates, &map, &function, threshold,
                  &path_client, &posToEsclude, record, move_base_local_costmap_topic_name);
              targetPos = std::make_pair(int(target.getX()), int(target.getY()));

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
//                  cout << "[pure_navigation.cpp@main] travelledDistance = " << travelledDistance << endl;
                  nav_utils.updatePathMetrics(
                      &count, &target, &previous, actualPose, &nearCandidates,
                      &graph2, &map, &function, &tabuList, &posToEsclude,
                      &history, encodedKeyValue, &numConfiguration,
                      &totalAngle, &travelledDistance, &numOfTurning, scanAngle,
                      &path_client, backTracking, robot_radius);
//                  cout << "[pure_navigation.cpp@main] travelledDistance = " << travelledDistance << endl;
                }
              }
              // ...otherwise, if the selected cell has already been visited
              else {
                //                                cout << "3" << endl;
                // If the graph is empty, stop the navigation
                if (graph2.size() == 0) break;
                cout << "   NearCandidates: " << graph2.at(graph2.size() - 1).second.size() << endl;
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
                  cout << "[main] candidateposition before: " << nearCandidates.size() << endl;
                  nav_utils.cleanPossibleDestination2(&nearCandidates, target);
                  nav_utils.cleanDestinationFromTabulist(&nearCandidates, &posToEsclude);
                  cout << "[main] candidateposition after: " << nearCandidates.size() << endl;
                  // Get the list of new candidate position with associated
                  // evaluation
                  record = function.evaluateFrontiers(&nearCandidates, &map,
                                                      threshold, &path_client);
                  // If there are candidate positions
                  // cout << "PoseToEsclude:" << endl;
                  // for (auto iter = posToEsclude.begin(); iter != posToEsclude.end(); iter++) {
                  //     cout << " " << iter->first << "," << iter->second << endl;
                  // }
                  // cout << "Candidates:" << endl;
                  // for (auto iter = nearCandidates.begin(); iter != nearCandidates.end(); iter++) {
                  //   cout << " " << iter->getX() << "," << iter->getY() << endl;
                  // }
                  while (1) {
                    if (record->size() != 0) {
                                  // if (nearCandidates.size() != 0) {
                      // Select the new RANDOM pose of the robot
                      target = getRandomFrontier(nearCandidates);
                      targetPos = make_pair(int(target.getX()), int(target.getY()));
                      cout << "   TargetPos: " << targetPos.first << ", " << targetPos.second << endl;
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
                        nav_utils.cleanDestinationFromTabulist(&nearCandidates, &posToEsclude);
                        // Get the list of new candidate position with
                        // associated evaluation
                        record = function.evaluateFrontiers(&nearCandidates, &map, threshold, &path_client);
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
                      cout << "nearCandidates before: " <<nearCandidates.size() << endl;
                      nav_utils.cleanPossibleDestination2(&nearCandidates, target);
                      nav_utils.cleanDestinationFromTabulist(&nearCandidates, &posToEsclude);
                      cout << "nearCandidates after: " <<nearCandidates.size() << endl;
                      record = function.evaluateFrontiers(&nearCandidates, &map, threshold, &path_client);
                      cout << "record: " << record->size() << endl;
                    }
                  }
                  cout << "[BT1-2]Target: " << target.getX() << ", " << target.getY() << endl;
                  backTracking = true;
                  previous = nav_utils.getCurrentPose(resolution, costresolution, &map, initFov, initRange);
                  success = nav_utils.showMarkerandNavigate(target, &marker_pub, &path,
                                        &path_client, &tabuList, &posToEsclude, min_robot_speed, robot_radius);
                  if (success == true)
                  {
//                    cout << "[pure_navigation.cpp@main] travelledDistance = " << travelledDistance << endl;
                    nav_utils.updatePathMetrics(
                        &count, &target, &previous, actualPose, &nearCandidates,
                        &graph2, &map, &function, &tabuList, &posToEsclude,
                        &history, encodedKeyValue, &numConfiguration,
                        &totalAngle, &travelledDistance, &numOfTurning, scanAngle,
                        &path_client, backTracking, robot_radius);
//                    cout << "[pure_navigation.cpp@main] travelledDistance = " << travelledDistance << endl;
                  }
                }
                // ... if the graph still does not present anymore candidate
                // positions for its last pose
                else {
                  cout << "[BT2 - Tabulist]There are visible cells but the "
                          "selected one is already "
                          "explored! Come back to two positions ago"
                       << endl;
                  cout << "Graph_size: " << graph2.size() << endl;
                  // Remove the last element (cell and associated candidate from
                  // there) from the graph
                  if (graph2.size() == 1) break;
                  graph2.pop_back();
                  // Select as new target, the new last element of the graph
                  string targetString = graph2.at(graph2.size() - 1).first;
                  nearCandidates = graph2.at(graph2.size() - 1).second;
                  cout << "nearCandidates before: " << nearCandidates.size() << endl;
                  nav_utils.cleanPossibleDestination2(&nearCandidates, target);
                  nav_utils.cleanDestinationFromTabulist(&nearCandidates, &posToEsclude);
                  cout << "nearCandidates after: " << nearCandidates.size() << endl;
                  target = record->getPoseFromEncoding(targetString);
                  // Save it history as cell visited more than once
                  history.push_back(function.getEncodedKey(target, 2));

                  count = count + 1;
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
              nav_utils.cleanPossibleDestination2(&nearCandidates, target);
              nav_utils.cleanDestinationFromTabulist(&nearCandidates, &posToEsclude);
              // Remove it from the graph
              graph2.pop_back();
              target = record->getPoseFromEncoding(targetString);
              cout << "--> " << function.getEncodedKey(target, 0) << endl;
//              cout << "Previous: " << function.getEncodedKey(previous, 2)
//                   << endl;
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
                nav_utils.cleanPossibleDestination2(&nearCandidates, target);
                nav_utils.cleanDestinationFromTabulist(&nearCandidates, &posToEsclude);
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
          cout << "Target: " << target.getX() << ", " << target.getY() << endl;
          x = target.getX();
          y = target.getY();
          orientation = roundf(target.getOrientation() * 100) / 100;
          range = target.getRange();
          FOV = target.getFOV();
          string actualPose = function.getEncodedKey(target, 0);
          map.setCurrentPose(target);
          // NOTE; calculate path and turnings between actual position and goal
          string encoding = to_string(target.getX()) + to_string(target.getY());
          visitedCell.emplace(encoding, 0);
          // Calculate how much time it takes to scan the current area
          target.setScanAngles(map.getSensingTime(x, y, orientation, FOV, range));
          // Get the scanning angle
          double scanAngle = target.getScanAngles().second - target.getScanAngles().first;
          // Update the overall scanned angle
          totalAngle += scanAngle;
          // ...and the overall scan time
          totalScanTime += nav_utils.calculateScanTime(scanAngle * 180 / M_PI);
          // Remove the current pose from the list of possible candidate cells
          cout << "NearCandidates before cleaning: " << nearCandidates.size() << endl;
          nav_utils.cleanPossibleDestination2(&nearCandidates, target);
          nav_utils.cleanDestinationFromTabulist(&nearCandidates, &posToEsclude);
          cout << "NearCandidates after cleaning: " << nearCandidates.size() << endl;
          // Get the list of the candidate cells with their evaluation
          EvaluationRecords *record = function.evaluateFrontiers(&nearCandidates, &map, threshold, &path_client);
          // cout << "Record obtained, size is " << record->size() << endl;

          // If there are candidate cells
                        // if (nearCandidates.size() > 0) {
          if (record->size() > 0) {
            // Find the new RANDOM destination
            target = getRandomFrontier(nearCandidates);
            targetPos = make_pair(int(target.getX()), int(target.getY()));

            // If this cells has not been visited before
            //            if ( ! contains ( tabuList,target ) )
            if ((!nav_utils.containsPos(&posToEsclude, targetPos))) {
              cout << "Selected target has not been visited YET! Going there now...." << endl;

              // Add it to the list of visited cells as first-view
              encodedKeyValue = 1;
              backTracking = true;
              // Update the current pose
              previous = nav_utils.getCurrentPose(resolution, costresolution, &map, initFov, initRange);
              success = nav_utils.showMarkerandNavigate(target, &marker_pub, &path, &path_client,
                                                        &tabuList, &posToEsclude, min_robot_speed, robot_radius);
              if (success == true){
//                cout << "[pure_navigation.cpp@main] travelledDistance = " << travelledDistance << endl;
                nav_utils.updatePathMetrics(
                    &count, &target, &previous, actualPose, &nearCandidates,
                    &graph2, &map, &function, &tabuList, &posToEsclude,
                    &history, encodedKeyValue,  &numConfiguration,
                    &totalAngle, &travelledDistance, &numOfTurning, scanAngle,
                    &path_client, backTracking, robot_radius);
//                cout << "[pure_navigation.cpp@main] travelledDistance = " << travelledDistance << endl;
              }
              // Leave the backtracking branch
              btMode = false;
              nearCandidates.clear();
              cout << "   [BT-MODE4] Go back to the best frontiers from the "
                      "previous positions in the graph"
                   << endl;
              cout << "       " << function.getEncodedKey(target, 0) << endl;
            }
              // ... otherwise, if the cells has already been visisted
            else {
              cout << "The selected target has ALREADY been visited! " << endl;
              // If there are other candidates
              if (nearCandidates.size() != 0) {
                cout << "   [BT-MODE1]Already visited, but there are other "
                        "candidates"
                     << endl;

                // Remove the destination from the candidate list
                nav_utils.cleanPossibleDestination2(&nearCandidates, target);
                nav_utils.cleanDestinationFromTabulist(&nearCandidates, &posToEsclude);
                // Select the new RANDOM destination
                target = getRandomFrontier(nearCandidates);
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
                nav_utils.cleanPossibleDestination2(&nearCandidates, target);
                nav_utils.cleanDestinationFromTabulist(&nearCandidates, &posToEsclude);
                // And remove from the graph
                graph2.pop_back();
                          // target = record.getPoseFromEncoding(targetString);
                target = record->getPoseFromEncoding(targetString);
                // Add it to the history of cell as already more than once
                encodedKeyValue = 2;
                // Leave backtracking
                btMode = true;
                // Clear candidate list
                nearCandidates.clear();
                cout << "   [BT-MODE2] No more candidates. Go back to previous positions in the graph"
                     << endl;
                cout << "       " << targetString << endl;
              }
            }
          }
          // ... if there are not candidate cells
          else {
            // Select as new pose, the last cell in the graph
            while (1) {
              // If there are still position on which doing backtracking
              if (graph2.size() != 0) {
                // If there are still frontiers from the current position
                if ((graph2.at(graph2.size() - 1).second).size() > 0)
                {
                  string targetString = graph2.at(graph2.size() - 1).first;
                  nearCandidates = graph2.at(graph2.size() - 1).second;
                  nav_utils.cleanPossibleDestination2(&nearCandidates, target);
                  nav_utils.cleanDestinationFromTabulist(&nearCandidates, &posToEsclude);
                              // target = record.getPoseFromEncoding(targetString);
                  target = record->getPoseFromEncoding(targetString);
                  // and the remove it form the graph
                  graph2.pop_back();
                  std::pair <float, float> tmp_position (target.getX(), target.getY());
                  // Leave backtracking if the selected cell does not appeat in the forbidden position list
//                  btMode = ! nav_utils.containsPos(&posToEsclude, tmp_position);
                  btMode = true;
                  cout << "   [BT-MODE3] There are no candidate frontiers in the "
                          "record. Go back to previous positions in the graph"
                       << endl;
                  cout << "       " << targetString << endl;
                  break;
                } else {
                  cout << "   [BT-MODE3] No more frontiers with the associate "
                          "position in the graph"
                       << endl;
                  graph2.pop_back();
                }
              } else {
                cout << "   [BT-MODE3] Graph2 is empty. Navigation is finished"
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
      // Plotting utilities
      map.drawVisitedCells();
      map.printVisitedCells(history);
      //        map.drawRFIDScan();
      //        map.drawRFIDGridScan(myGrid);
      //        myGrid.saveAs(("/home/pulver/Desktop/MCDM/rfid_result_gridmap.pgm"));

      cout << "Num configuration: " << numConfiguration << endl;
      cout << "Travelled distance calculated during the algorithm: "
           << travelledDistance << endl;

      cout << "------------------ HISTORY -----------------" << endl;
      // Calculate which cells have been visited only once
      list<Pose> tmp_history = nav_utils.cleanHistory(&history, &record);
      nav_utils.calculateDistance(tmp_history, &path_client, robot_radius);

      cout << "------------------ TABULIST -----------------" << endl;
      nav_utils.calculateDistance(tabuList, &path_client, robot_radius);

      // Trasform distance in meters
      if (resolution ==
          1.0) // Corridor map has a resolution of 0.5 meter per cell
      {
        travelledDistance = travelledDistance / 2;
      }

      nav_utils.printResult(newSensedCells, totalFreeCells, precision,
                            numConfiguration, travelledDistance, numOfTurning,
                            totalAngle, totalScanTime, resolution,
                            w_info_gain, w_travel_distance, w_sensing_time, out_log);

      // Find the tag
      //        std::pair<int,int> tag = map.findTag();
      //        cout << "RFID pose: [" << tag.second << "," << tag.first << "]"
      //        << endl;
      //        tag = map.findTagfromGridMap(myGrid);
      //        cout << "[Grid]RFID pose: [" << tag.second << "," << tag.first
      //        << "]" << endl;
      cout
          << "-----------------------------------------------------------------"
          << endl;
      auto endMCDM = ros::Time::now().toSec();;

      double totalTimeMCDM = endMCDM - startMCDM;
      cout << "Total time for MCDM algorithm : " << totalTimeMCDM << "s, "
           << totalTimeMCDM / 60 << " m " << endl;
      cout << "Spinning at the end" << endl;

      // Stop recording the bag
      srv_rosbag.request.cmd = "stop";
      if (rosbag_client.call(srv_rosbag)){
        cout << "Stop recording the bag..." << endl;
        sleep(5);
      }else{
        cout << "Error occurring while stop recording the bag. Exiting now!" << endl;
      }

      sleep(1);
      ros::shutdown();
    }

    ros::spinOnce();
    r.sleep();

  } // end while ros::ok
} // end main


void grid_callback(const nav_msgs::OccupancyGridConstPtr &msg) {
  // printf("RECEIVED A MAP!");
  if (costmapReceived == 0) {
    cout << "CALLBACK FIRST!" << endl;
    costmap_grid = *msg;
    costresolution = msg->info.resolution;
    costwidth = msg->info.width;
    costheight = msg->info.height;
    costorigin = msg->info.origin;
    for (int i = 0; i < msg.get()->data.size(); ++i) {
      occdata.push_back(msg->data.at(i));
    }
    std::cout << "size of occdata " << occdata.size()
              << " size of message data " << msg->data.size() << std::endl;
    std::cout << "height " << msg->info.height << " width " << msg->info.width
              << " resolution " << msg->info.resolution << std::endl;
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
  private_node_handle.param("rosbag_srv_name", rosbag_srv_name, std::string("/record/cmd"));

}

void printROSParams(){
  printf("/////////////////////////////////////////////////////////////////////////\n");
  printf("[pure_navigation@printROSParams] Using the following ros params:\n");
  printf("   - robot_radius [%3.3f]\n",  robot_radius);
  printf("   - static_map_srv_name [%s]\n", static_map_srv_name.c_str());
  printf("   - make_plan_srv_name [%s]\n", make_plan_srv_name.c_str());
  printf("   - move_base_goal_topic_name [%s]\n", move_base_goal_topic_name.c_str());
  printf("   - move_base_srv_name [%s]\n", move_base_srv_name.c_str());
  printf("   - nav_grid_debug_topic_name [%s]\n", nav_grid_debug_topic_name.c_str());
  printf("   - planning_grid_debug_topic_name [%s]\n", planning_grid_debug_topic_name.c_str());
  printf("   - move_base_costmap_topic_name [%s]\n", move_base_costmap_topic_name.c_str());
  printf("   - move_base_costmap_updates_topic_name [%s]\n", move_base_costmap_updates_topic_name.c_str());
  printf("   - marker_pub_topic_name [%s]\n", marker_pub_topic_name.c_str());
  printf("/////////////////////////////////////////////////////////////////////////\n");

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


  // create subscribers, only when we are sure the right people is publishing
//  printf("[pure_navigation@createROSComms] Waiting for move_base action server to come up");
//  MoveBaseClient ac(move_base_srv_name, true);
//  while (!ac.waitForServer(ros::Duration(5.0))) {
//    printf("[pure_navigation@createROSComms]... waiting ...");
//  }
  tag_coverage_sub = nh.subscribe<std_msgs::Float32>("/tag_coverage", 10, tag_coverage_callback);


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

Pose getRandomFrontier(list<Pose> nearCandidates){
  long tot_candidates = nearCandidates.size();
  std::random_device rd;     // only used once to initialise (seed) engine
  std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
  std::uniform_int_distribution<int> uni(0, tot_candidates-1); // guaranteed unbiased
  std::vector<Pose> v{ std::make_move_iterator(std::begin(nearCandidates)),
                    std::make_move_iterator(std::end(nearCandidates)) };
  auto random_integer = uni(rng);
  return v.at(random_integer);
}

void tag_coverage_callback(const std_msgs::Float32 msg){
  tag_coverage_percentage = msg.data;
}
