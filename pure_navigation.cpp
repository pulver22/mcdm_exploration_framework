#include "Criteria/traveldistancecriterion.h"
#include "PathFinding/astar.h"
#include "map.h"
#include "mcdmfunction.h"
#include "newray.h"
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
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// mfc ...
#include <ros/console.h>
// mfc ...

using namespace std;
using namespace dummy;

bool contains(std::list<Pose> &list, Pose &p);

bool containsPos(std::list<std::pair<float, float> > positionEscluded,
                 std::pair<float, float> p);

void cleanPossibleDestination2(std::list<Pose> *possibleDestinations, Pose &p);

void pushInitialPositions(dummy::Map map, float x, float y, float orientation,
                          int range, int FOV, double threshold,
                          string actualPose,
                          vector<pair<string, list<Pose> >> *graph2,
                          ros::ServiceClient *path_client);

double calculateScanTime(double scanAngle);

void calculateDistance(list<Pose> list, dummy::Map &map, Astar *astar);

Pose createFromInitialPose(Pose pose, float variation, int range, int FOV);

void updatePathMetrics(
    int *count, Pose *target, Pose *previous, string actualPose,
    list<Pose> *nearCandidates, vector<pair<string, list<Pose> >> *graph2,
    dummy::Map *map, MCDMFunction *function, list<Pose> *tabuList,
    list<pair<float, float> > *posToEsclude, vector<string> *history,
    int encodedKeyValue, Astar *astar, long *numConfiguration,
    double *totalAngle, double *travelledDistance, int *numOfTurning,
    double scanAngle, ros::ServiceClient *path_client, bool backTracking);

list<Pose> cleanHistory(vector<string> *history,
                        EvaluationRecords *record_history);

void printResult(long newSensedCells, long totalFreeCells, double precision,
                 long numConfiguration, double travelledDistance,
                 int numOfTurning, double totalAngle, double totalScanTime);

void showMarkerandNavigate(Pose target, ros::Publisher *marker_pub,
                           nav_msgs::GetPlan *path,
                           ros::ServiceClient *path_client,
                           list<Pose> *tabuList,
                           std::list<std::pair<float, float> > *posToEsclude);

// ROS varies
void move(float x, float y, float orientation, float time_travel,
          list<Pose> *tabuList,
          std::list<std::pair<float, float> > *posToEsclude);

void update_callback(const map_msgs::OccupancyGridUpdateConstPtr &msg);

void grid_callback(const nav_msgs::OccupancyGridConstPtr &msg);

Pose getCurrentPose(float resolution, float costresolution, dummy::Map *map,
                    double initFov, int initRange);

double getPathLen(std::vector<geometry_msgs::PoseStamped> poses);

void printROSParams();

void loadROSParams();
void createROSComms();

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;
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

//  ROS PARAMETERS ....................................
std::string static_map_srv_name;
std::string make_plan_srv_name;
std::string move_base_goal_topic_name;
std::string move_base_srv_name;
std::string nav_grid_debug_topic_name;
std::string planning_grid_debug_topic_name;
std::string move_base_costmap_topic_name;
std::string move_base_costmap_updates_topic_name;
std::string  marker_pub_topic_name;

// Ros services/subscribers/publishers
ros::ServiceClient map_service_client_;
ros::ServiceClient path_client;
nav_msgs::GetMap srv_map;
ros::Publisher moveBasePub;
ros::Subscriber costmap_sub;
ros::Subscriber costmap_update_sub;
ros::Publisher gridPub;
ros::Publisher planningPub;
ros::Publisher marker_pub;

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
    ROS_INFO("Parameters:\n- Field of View (%3.3f)\n- Sensing Range (%d)\n- "
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
  auto startMCDM = chrono::high_resolution_clock::now();
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
          getCurrentPose(resolution, costresolution, &map, initFov, initRange);
      Pose target = start_pose;
      Pose previous = target;
      Pose invertedInitial =
          createFromInitialPose(start_pose, M_PI, initRange, initFov);
      Pose eastInitial =
          createFromInitialPose(start_pose, M_PI / 2, initRange, initFov);
      Pose westInitial =
          createFromInitialPose(start_pose, 3 * M_PI / 2, initRange, initFov);
      std::pair<float, float> targetPos;
      long numConfiguration = 1;
      vector<pair<string, list<Pose> >> graph2;
      bool backTracking = false;
      NewRay ray;
      ray.setGridToPathGridScale(gridToPathGridScale);
      MCDMFunction function;
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
      list<Pose> unexploredFrontiers;
      list<Pose> tabuList;
      std::list<std::pair<float, float> > posToEsclude;
      list<Pose> nearCandidates;
      EvaluationRecords record;
      Astar astar;
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


      do {

        cout << "\n============================================" << endl;
        cout << "New iteration, position: " << target.getX() << ","
             << target.getY() << endl;
        cout << "Graph size: " << graph2.size() << endl;
        if (graph2.size() == 0 and count != 0)
          break;
        for (auto it = graph2.begin(); it != graph2.end(); it++) {
          cout << " " << it->first << endl;
        }

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
          Pose target = getCurrentPose(resolution, costresolution, &map,
                                       initFov, initRange);
          //          cout << "[Current Pose]: " << target.getX() << ", " <<
          //          target.getY()<<" m. " << ", " << target.getOrientation()
          //          << "("<< (target.getOrientation() * 180 / M_PI) <<" deg),
          //          " << target.getFOV() <<", " << target.getRange() << endl;
          //            cout << "[Tmp]: " << tmp_target.getY() << ", " <<
          //            tmp_target.getX() << ", " <<
          //            (tmp_target.getOrientation() + 360 ) % 360 << ", " <<
          //            tmp_target.getFOV() <<", " << tmp_target.getRange() <<
          //            endl;
          map.getPathPlanningIndex(target.getX(), target.getY(), i, j);
          //          cout << "[Current CELL INDEX in PathPlanningGrid]: " << i
          //          << ", " << j << endl;
          map.getPathPlanningPosition(targetX_meter, targetY_meter, i, j);
          //          cout << "[Current POSITION in PathPlanningGrid]: " <<
          //          targetX_meter << ", " << targetY_meter << endl;
          map.getGridIndex(target.getX(), target.getY(), i, j);
          //          cout << "[Current CELL INDEX in NavigationGrid]: " << i <<
          //          ", " << j << endl;

          //          newSensedCells = sensedCells + ray.performSensingOperation
          //          ( &map, 5.0, 3.0, 90, 180, 3, -1.8, 1.8 );
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
          totalScanTime += calculateScanTime(scanAngle * 180 / M_PI);
          // Calculare the relative RFID tag position to the robot position
          //            relTagCoord = map.getRelativeTagCoord(absTagX, absTagY,
          //            target.getX(), target.getY());
          // Calculate the received power and phase
          //            double rxPower = received_power_friis(relTagCoord.first,
          //            relTagCoord.second, freq, txtPower);
          //            double phase = phaseDifference(relTagCoord.first,
          //            relTagCoord.second, freq);
          // Update the path planning and RFID map
          //          cout << endl << "[pure_navigation.cpp]" <<
          //          map.getGridValue(target.getX() + 1, target.getY() + 1) <<
          //          endl;

          map.getPathPlanningIndex(x, y, cell_i, cell_j);
          //          range = range / costresolution;
          //          cout << "[pure_navigation.cpp@main](x,y) = (" << x << ","
          //          << y << ")" << endl;
          map.updatePathPlanningGrid(x, y, range);
          //            myGrid.addEllipse(rxPower - SENSITIVITY,
          //            map.getNumGridCols() - target.getX(),  target.getY(),
          //            target.getOrientation(), -0.5, 7.0);
          // Search for new candidate position
          //          map.getGridIndex(x, y, cell_i, cell_j);

          gridPub.publish(map.toMessageGrid());
          planningPub.publish(map.toMessagePathPlanning());
          map.plotPathPlanningGridColor("/tmp/pathplanning_lastLoop.png");
          map.plotGridColor("/tmp/nav_lastLoop.png");

          map.findCandidatePositions(x, y, orientation, FOV, range);
          //          ray.findCandidatePositions(&map, x, y, orientation, FOV,
          //          range);
          vector<pair<float, float> > candidatePosition =
              map.getCandidatePositions();

          map.emptyCandidatePositions();
          cout << " Candidate cleaned!" << endl;
          //          for (auto it = candidatePosition.begin(); it !=
          //          candidatePosition.end(); it++) {
          //            cout << (*it).first << "/" << (*it).second << endl;
          //          }
          cout << "Size of initial candidate postions: "
               << candidatePosition.size() << endl;

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
            pushInitialPositions(map, x, y, orientation, range, FOV, threshold,
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
              //                            cout << "       " << targetString <<
              //                            endl;
              cout << "       " << function.getEncodedKey(target, 0) << endl;
              count = count + 1;
              btMode = true;
              //                            backTracking = true;
              //                            updatePathMetrics(&count, &target,
              //                            &previous, actualPose,
              //                                              &nearCandidates,
              //                                              &graph2, &map,
              //                                              &function,
              //                                              &tabuList,
              //                                              &posToEsclude,
              //                                              &history,
              //                                              encodedKeyValue,
              //                                              &astar,
              //                                              &numConfiguration,
              //                                              &totalAngle,
              //                                              &travelledDistance,
              //                                              &numOfTurning,
              //                                              scanAngle,
              //                                              &path_client,
              //                                              backTracking);
              //                            showMarkerandNavigate(target,
              //                            &marker_pub, &path, &path_client,
              //                            &tabuList,
              //                                                  &posToEsclude);
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
              list<Pose> tmp_history = cleanHistory(&history, &record);
              calculateDistance(tmp_history, map, &astar);

              cout << "------------------ TABULIST -----------------" << endl;
              // Calculate the path connecting the cells in the tabulist, namely
              // the cells that are visited one time and couldn't be visite
              // again
              calculateDistance(tabuList, map, &astar);

              // Normalise the travel distance in meter
              // NOTE: assuming that the robot is moving at 0.5m/s and the
              // resolution of the map is 0.5m per cell)
              if (resolution == 1.0) {
                travelledDistance = travelledDistance / 2;
              }
              printResult(newSensedCells, totalFreeCells, precision,
                          numConfiguration, travelledDistance, numOfTurning,
                          totalAngle, totalScanTime);
              auto endMCDM = chrono::high_resolution_clock::now();
              double totalTimeMCDM =
                  chrono::duration<double, milli>(endMCDM - startMCDM).count();
              cout << "Total time for MCDM algorithm : " << totalTimeMCDM
                   << "ms, " << totalTimeMCDM / 1000 << " s, "
                   << totalTimeMCDM / 60000 << " m " << endl;
              cout << "Total time in empirical way : "
                   << travelledDistance / 0.25 + timeOfScanning / 1000 << endl;
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
              Pose p2 = Pose((*it).first, (*it).second,
                             roundf(M_PI / 4 * 100) / 100, range, FOV);
              Pose p3 = Pose((*it).first, (*it).second,
                             roundf(M_PI / 2 * 100) / 100, range, FOV);
              Pose p4 = Pose((*it).first, (*it).second,
                             roundf(3 * M_PI / 4 * 100) / 100, range, FOV);
              Pose p5 = Pose((*it).first, (*it).second,
                             roundf(M_PI * 100) / 100, range, FOV);
              Pose p6 = Pose((*it).first, (*it).second,
                             roundf(5 * M_PI / 4 * 100) / 100, range, FOV);
              Pose p7 = Pose((*it).first, (*it).second,
                             roundf(3 * M_PI / 2 * 100) / 100, range, FOV);
              Pose p8 = Pose((*it).first, (*it).second,
                             roundf(7 * M_PI / 4 * 100) / 100, range, FOV);
              frontiers.push_back(p1);
              frontiers.push_back(p2);
              frontiers.push_back(p3);
              frontiers.push_back(p4);
              frontiers.push_back(p5);
              frontiers.push_back(p6);
              frontiers.push_back(p7);
              frontiers.push_back(p8);
            }
            //            cout << "Frontiers" << endl;
            //            for (auto it = frontiers.begin(); it !=
            //            frontiers.end(); it++) {
            //              cout << " " << record.getEncodedKey(*it) << endl;
            //            }
            unexploredFrontiers = frontiers;

            // Evaluate the frontiers and return a list of <frontier,
            // evaluation> pairs
            EvaluationRecords *record = function.evaluateFrontiers(
                frontiers, &map, threshold, &path_client);
            nearCandidates = record->getFrontiers();

            // Print the frontiers with the respective evaluation
            cout << "Number of frontiers identified: " << nearCandidates.size()
                 << endl;
            unordered_map<string, double> evaluation = record->getEvaluations();
            //            for (auto it = evaluation.begin(); it !=
            //            evaluation.end(); it++) {
            //              string tmp = (*it).first;
            //              double value = (*it).second;
            //              cout << tmp << " " << value << endl;
            //            }

            //            cout << "Size of record: " << record->size() << endl;

            // If there are candidate positions
            if (record->size() != 0) {
              // Set the previous pose equal to the current one (represented by
              // target)
              previous = target;
              // Select the new robot destination from the list of candidates

              std::pair<Pose, double> result = function.selectNewPose(record);
              target = result.first;
              // If the selected destination does not appear among the cells
              // already visited
              auto tabuList_it = tabuList.begin();
              //              cout << "Tabulist:" << endl;
              //              for ( tabuList_it; tabuList_it != tabuList.end();
              //              tabuList_it++ )
              //              {
              //                cout << record->getEncodedKey(*tabuList_it) <<
              //                endl;
              //              }
              //                            cout << "PoseToEsclude:" << endl;
              //                            for (auto iter =
              //                            posToEsclude.begin(); iter !=
              //                            posToEsclude.end(); iter++) {
              //                                cout << " " << iter->first <<
              //                                "," << iter->second << endl;
              //                            }
              targetPos = std::make_pair(target.getX(), target.getY());
              //                            cout << "1" << endl;
              //                          if ( ! contains ( tabuList,target ))
              if (!containsPos(posToEsclude, targetPos)) {
                //                                cout << "2" << endl;
                // Add it to the list of visited cells as first-view
                encodedKeyValue = 1;
                backTracking = false;
                updatePathMetrics(
                    &count, &target, &previous, actualPose, &nearCandidates,
                    &graph2, &map, &function, &tabuList, &posToEsclude,
                    &history, encodedKeyValue, &astar, &numConfiguration,
                    &totalAngle, &travelledDistance, &numOfTurning, scanAngle,
                    &path_client, backTracking);
                showMarkerandNavigate(target, &marker_pub, &path, &path_client,
                                      &tabuList, &posToEsclude);

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
                  cleanPossibleDestination2(&nearCandidates, target);
                  // Get the list of new candidate position with associated
                  // evaluation
                  record = function.evaluateFrontiers(nearCandidates, &map,
                                                      threshold, &path_client);
                  // If there are candidate positions
                  while (1) {
                    if (record->size() != 0) {
                      // Select the new pose of the robot
                      std::pair<Pose, double> result =
                          function.selectNewPose(record);
                      target = result.first;
                      targetPos = make_pair(target.getX(), target.getY());
                      //                      if (!contains(tabuList, target)) {
                      if (!containsPos(posToEsclude, targetPos)) {
                        // If the new selected position is not in the Tabulist

                        encodedKeyValue = 1;
                        //                                                backTracking
                        //                                                =
                        //                                                true;
                        //                                                updatePathMetrics(&count,
                        //                                                &target,
                        //                                                &previous,
                        //                                                actualPose,
                        //                                                                  &nearCandidates, &graph2, &map,
                        //                                                                  &function, &tabuList, &posToEsclude, &history,
                        //                                                                  encodedKeyValue, &astar, &numConfiguration,
                        //                                                                  &totalAngle,
                        //                                                                  &travelledDistance, &numOfTurning, scanAngle,
                        //                                                                  &path_client, backTracking);
                        //                                                showMarkerandNavigate(target,
                        //                                                &marker_pub,
                        //                                                &path,
                        //                                                &path_client,
                        //                                                &tabuList,
                        //                                                                      &posToEsclude);
                        //                                                btMode
                        //                                                =
                        //                                                true;
                        scan = false;
                        // Set that we are now in backtracking
                        cout << "[BT1] Break the while" << endl;
                        break; // the while loop
                      } else {
                        //                        cout << "Inside the while" <<
                        //                        endl;
                        // Remove the current position from possible candidates
                        cleanPossibleDestination2(&nearCandidates, target);
                        // Get the list of new candidate position with
                        // associated evaluation
                        record = function.evaluateFrontiers(
                            nearCandidates, &map, threshold, &path_client);
                      }
                    }
                    // If there are no more candidate position from the last
                    // position in the graph
                    else {
                      // if the graph is now empty, stop the navigation
                      if (graph2.size() == 0)
                        break;
                      // Otherwise, select as new position the last cell in the
                      // graph and then remove it from there
                      string targetString = graph2.at(graph2.size() - 1).first;
                      graph2.pop_back();
                      previous = target;
                      target = record->getPoseFromEncoding(targetString);
                      scan = false;
                      break;
                      cout << "[BT1] 2" << endl;
                    }
                  }
                  cout << "[BT1-2]Target: " << target.getX() << ", "
                       << target.getY() << endl;
                  backTracking = true;
                  updatePathMetrics(
                      &count, &target, &previous, actualPose, &nearCandidates,
                      &graph2, &map, &function, &tabuList, &posToEsclude,
                      &history, encodedKeyValue, &astar, &numConfiguration,
                      &totalAngle, &travelledDistance, &numOfTurning, scanAngle,
                      &path_client, backTracking);
                  showMarkerandNavigate(target, &marker_pub, &path,
                                        &path_client, &tabuList, &posToEsclude);

                  scan = true;
                }
                // ... if the graph still does not present anymore candidate
                // positions for its last pose
                else {
                  cout << "[BT2 - Tabulist]There are visible cells but the "
                          "selected one is already "
                          "explored! Come back to two position ago"
                       << endl;
                  // Remove the last element (cell and associated candidate from
                  // there) from the graph
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
              if (graph2.size() == 0)
                break;
              cout << "Graph size: " << graph2.size() << endl;
              for (auto it = graph2.begin(); it != graph2.end(); it++) {
                cout << " " << it->first << endl;
              }
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
                //                                backTracking = true;
                //                                updatePathMetrics(&count,
                //                                &target, &previous,
                //                                actualPose, &nearCandidates,
                //                                &graph2,
                //                                                  &map,
                //                                                  &function,
                //                                                  &tabuList,
                //                                                  &posToEsclude,
                //                                                  &history,
                //                                                  encodedKeyValue,
                //                                                  &astar,
                //                                                  &numConfiguration,
                //                                                  &totalAngle,
                //                                                  &travelledDistance,
                //                                                  &numOfTurning,
                //                                                  scanAngle,
                //                                                  &path_client,
                //                                                  backTracking);
                //                                showMarkerandNavigate(target,
                //                                &marker_pub, &path,
                //                                &path_client, &tabuList,
                //                                                      &posToEsclude);

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
          cout << "Previous: " << previous.getX() << ", " << previous.getY()
               << endl;
          cout << "Target: " << target.getX() << ", " << target.getY() << endl;
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
          target.setScanAngles(
              map.getSensingTime(x, y, orientation, FOV, range));
          // Get the scanning angle
          double scanAngle =
              target.getScanAngles().second - target.getScanAngles().first;
          // Update the overall scanned angle
          totalAngle += scanAngle;
          // ...and the overall scan time
          totalScanTime += calculateScanTime(scanAngle * 180 / M_PI);
          // Calculate the relative coordinate to the robot of the RFID tag
          //            relTagCoord = map.getRelativeTagCoord(absTagX, absTagY,
          //            target.getX(), target.getY());
          // Calculate received power and phase
          //            double rxPower = received_power_friis(relTagCoord.first,
          //            relTagCoord.second, freq, txtPower);
          //            double phase = phaseDifference(relTagCoord.first,
          //            relTagCoord.second, freq);
          //          cout << "[pure_navigation.cpp@main](x,y) = (" << x << ","
          //          << y << ")" << endl;
          map.updatePathPlanningGrid(x, y, range);
          //            myGrid.addEllipse(rxPower - SENSITIVITY,
          //            map.getNumGridCols() - target.getX(), target.getY(),
          //            target.getOrientation(), -0.5, 7.0);
          // Remove the current pose from the list of possible candidate cells
          cleanPossibleDestination2(&nearCandidates, target);
          cout << "Cleaned" << endl;
          // Get the list of the candidate cells with their evaluation
          EvaluationRecords *record = function.evaluateFrontiers(
              nearCandidates, &map, threshold, &path_client);
          cout << "Record obtained, size is " << record->size() << endl;

          // If there are candidate cells
          if (record->size() > 0) {
            // Find the new destination
            std::pair<Pose, double> result = function.selectNewPose(record);
            target = result.first;
            targetPos = make_pair(target.getX(), target.getY());

            // If this cells has not been visited before
            //            if ( ! contains ( tabuList,target ) )
            if (!containsPos(posToEsclude, targetPos)) {
              // Add it to the list of visited cells as first-view
              encodedKeyValue = 1;
              backTracking = true;
              updatePathMetrics(&count, &target, &previous, actualPose,
                                &nearCandidates, &graph2, &map, &function,
                                &tabuList, &posToEsclude, &history,
                                encodedKeyValue, &astar, &numConfiguration,
                                &totalAngle, &travelledDistance, &numOfTurning,
                                scanAngle, &path_client, backTracking);
              showMarkerandNavigate(target, &marker_pub, &path, &path_client,
                                    &tabuList, &posToEsclude);
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
                cleanPossibleDestination2(&nearCandidates, target);
                // Get the candidates with their evaluation
                EvaluationRecords *record = function.evaluateFrontiers(
                    nearCandidates, &map, threshold, &path_client);
                // Select the new destination
                std::pair<Pose, double> result = function.selectNewPose(record);
                target = result.first;

                // Add it to the list of visited cells as first-view
                encodedKeyValue = 1;
                backTracking = true;
                updatePathMetrics(
                    &count, &target, &previous, actualPose, &nearCandidates,
                    &graph2, &map, &function, &tabuList, &posToEsclude,
                    &history, encodedKeyValue, &astar, &numConfiguration,
                    &totalAngle, &travelledDistance, &numOfTurning, scanAngle,
                    &path_client, backTracking);
              }
              // ...otherwise, if there are no more candidates
              else {

                // Select as target the last element in the graph
                string targetString = graph2.at(graph2.size() - 1).first;
                nearCandidates = graph2.at(graph2.size() - 1).second;
                // And remove from the graph
                graph2.pop_back();
                target = record->getPoseFromEncoding(targetString);
                // Add it to the history of cell as already more than once
                encodedKeyValue = 2;
                //                                backTracking = true;
                //                                updatePathMetrics(&count,
                //                                &target, &previous,
                //                                actualPose, &nearCandidates,
                //                                &graph2,
                //                                                  &map,
                //                                                  &function,
                //                                                  &tabuList,
                //                                                  &posToEsclude,
                //                                                  &history,
                //                                                  encodedKeyValue,
                //                                                  &astar,
                //                                                  &numConfiguration,
                //                                                  &totalAngle,
                //                                                  &travelledDistance,
                //                                                  &numOfTurning,
                //                                                  scanAngle,
                //                                                  &path_client,
                //                                                  backTracking);
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

                  // Add it in history as cell visited more than once
                  //                            encodedKeyValue = 2;
                  //                        backTracking = true;
                  //                        updatePathMetrics(&count, &target,
                  //                        &previous, actualPose,
                  //                        &nearCandidates, &graph2, &map,
                  //                                          &function,
                  //                                          &tabuList,
                  //                                          &posToEsclude,
                  //                                          &history,
                  //                                          encodedKeyValue,
                  //                                          &astar,
                  //                                          &numConfiguration,
                  //                                          &totalAngle,
                  //                                          &travelledDistance,
                  //                                          &numOfTurning,
                  //                                          scanAngle,
                  //                                          &path_client,
                  //                                          backTracking);
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
      list<Pose> tmp_history = cleanHistory(&history, &record);
      calculateDistance(tmp_history, map, &astar);

      cout << "------------------ TABULIST -----------------" << endl;
      calculateDistance(tabuList, map, &astar);

      // Trasform distance in meters
      if (resolution ==
          1.0) // Corridor map has a resolution of 0.5 meter per cell
      {
        travelledDistance = travelledDistance / 2;
      }

      printResult(newSensedCells, totalFreeCells, precision, numConfiguration,
                  travelledDistance, numOfTurning, totalAngle, totalScanTime);
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
      auto endMCDM = chrono::high_resolution_clock::now();

      double totalTimeMCDM =
          chrono::duration<double, milli>(endMCDM - startMCDM).count();
      cout << "Total time for MCDM algorithm : " << totalTimeMCDM << "ms, "
           << totalTimeMCDM / 1000 << " s, " << totalTimeMCDM / 60000 << " m "
           << endl;
      cout << "Spinning at the end" << endl;
      sleep(1);
      exit(0);
    }

    ros::spinOnce();
    r.sleep();

  } // end while ros::ok
} // end main

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

bool containsPos(std::list<std::pair<float, float> > positionEscluded,
                 std::pair<float, float> p) {
  bool result = false;
  MCDMFunction function;

  auto findIter =
      std::find(positionEscluded.begin(), positionEscluded.end(), p);
  if (findIter != positionEscluded.end()) {
    result = true;
  }

  return result;
}

void cleanPossibleDestination2(std::list<Pose> *possibleDestinations, Pose &p) {
  MCDMFunction function;
  //  cout << "[pure_navigation.cpp@cleanPossibleDestination2] I want to remove
  //  "<< function.getEncodedKey(p,0) << endl;
  //  cout << "[pure_navigation.cpp@cleanPossibleDestination2] Size possible
  //  destination: " << possibleDestinations->size() << endl;
  //  for (auto it = possibleDestinations->begin(); it !=
  //  possibleDestinations->end(); it++)
  //  {
  //    cout << function.getEncodedKey(*it, 0) << endl;
  //  }
  std::list<Pose>::iterator findIter =
      std::find(possibleDestinations->begin(), possibleDestinations->end(), p);
  if (findIter != possibleDestinations->end()) {
    //    cout << "[pure_navigation.cpp@cleanPossibleDestination2] EncodedKey:"
    //    <<  function.getEncodedKey(*findIter,0) << "\n" << endl;
    possibleDestinations->erase(findIter);
  }
  //  else cout<< "[pure_navigation.cpp@cleanPossibleDestination2] Cell not
  //  found\n" << endl;

  //  for (auto it = possibleDestinations->begin(); it !=
  //  possibleDestinations->end(); it++)
  //  {
  //    cout << function.getEncodedKey(*it, 0) << endl;
  //  }
}

void pushInitialPositions(dummy::Map map, float x, float y, float orientation,
                          int range, int FOV, double threshold,
                          string actualPose,
                          vector<pair<string, list<Pose> >> *graph2,
                          ros::ServiceClient *path_client) {
  NewRay ray;
  MCDMFunction function;
  map.findCandidatePositions(x, y, orientation, FOV, range);
  vector<pair<float, float> > candidatePosition = map.getCandidatePositions();
  map.emptyCandidatePositions();
  list<Pose> frontiers;
  vector<pair<float, float> >::iterator it = candidatePosition.begin();
  for (it; it != candidatePosition.end(); it++) {
    Pose p1 = Pose((*it).first, (*it).second, 0, range, FOV);
    Pose p2 = Pose((*it).first, (*it).second, 180, range, FOV);
    Pose p3 = Pose((*it).first, (*it).second, 90, range, FOV);
    Pose p4 = Pose((*it).first, (*it).second, 270, range, FOV);
    frontiers.push_back(p1);
    frontiers.push_back(p2);
    frontiers.push_back(p3);
    frontiers.push_back(p4);
  }
  EvaluationRecords *record =
      function.evaluateFrontiers(frontiers, &map, threshold, path_client);
  list<Pose> nearCandidates = record->getFrontiers();
  cout << "Number of candidates:" << nearCandidates.size() << endl;
  std::pair<string, list<Pose> > pair = make_pair(actualPose, nearCandidates);
  graph2->push_back(pair);
}

double calculateScanTime(double scanAngle) {
  return (-7.2847174296449998e-006 * scanAngle * scanAngle * scanAngle +
          2.2131847908245512e-003 * scanAngle * scanAngle +
          1.5987873410233613e-001 * scanAngle + 10);
}

Pose createFromInitialPose(Pose pose, float variation, int range, int FOV) {
  Pose tmp = Pose(pose.getX(), pose.getY(), pose.getOrientation() + variation,
                  FOV, range);
  return tmp;
}

void calculateDistance(list<Pose> history, dummy::Map &map, Astar *astar) {
  std::list<Pose>::iterator it = history.begin();
  double travelledDistance = 0;
  int numOfTurning = 0;
  // Calculate the overall path connecting these cells
  for (it; it != prev(history.end(), 1); it++) {
    //        cout << function->getEncodedKey(*it,1) << endl; // print cell in
    //        the tabulist
    std::list<Pose>::iterator it2 = next(it, 1);
    string path = astar->pathFind((*it2).getX(), (*it2).getY(), (*it).getX(),
                                  (*it).getY(), &map);
    travelledDistance = travelledDistance + astar->lenghtPath(path);
    numOfTurning = numOfTurning + astar->getNumberOfTurning(path);
    // cout << astar.lengthPath ( path ) << endl;
  }
  cout << "Number of cells: " << history.size() << endl;
  cout << "Num of Turning: " << numOfTurning << endl;
  cout << "Travelled distance (cells): " << travelledDistance << endl;
  cout << "Travelled distance (meters): " << travelledDistance / 2.0
       << endl; // Valid only if resolution == 1.0 (cell side is 0.5m)
}

void updatePathMetrics(
    int *count, Pose *target, Pose *previous, string actualPose,
    list<Pose> *nearCandidates, vector<pair<string, list<Pose> >> *graph2,
    dummy::Map *map, MCDMFunction *function, list<Pose> *tabuList,
    list<pair<float, float> > *posToEsclude, vector<string> *history,
    int encodedKeyValue, Astar *astar, long *numConfiguration,
    double *totalAngle, double *travelledDistance, int *numOfTurning,
    double scanAngle, ros::ServiceClient *path_client, bool backTracking) {

  nav_msgs::GetPlan path;
  double path_len;
  // Add it to the list of visited cells as first-view
  history->push_back(function->getEncodedKey(*target, encodedKeyValue));
  //  cout << function->getEncodedKey ( *target,1 ) << endl;
  // Add it to the list of visited cells from which acting
  tabuList->push_back(*target);
  posToEsclude->push_back(make_pair(roundf(((*target).getX() * 100) / 100),
                                    roundf((*target).getY() * 100 / 100)));
  // Remove it from the list of candidate position
  cleanPossibleDestination2(nearCandidates, *target);
  // Push in the graph the previous robot pose and the new list of candidate
  // position, without the current pose of the robot
  // We don't want to visit this cell again
  if (backTracking == false) {
    std::pair<string, list<Pose> > pair = make_pair(actualPose, *nearCandidates);
    graph2->push_back(pair);
  }

  // Calculate the path from the previous robot pose to the current one
  //  string path = astar->pathFind ( target->getX(), target->getY(),
  //  previous->getX(), previous->getY(), map );
  path.request.start.header.frame_id = "map";
  path.request.start.pose.position.x = previous->getX();
  path.request.start.pose.position.y = previous->getY();
  path.request.start.pose.orientation.w = 1;
  path.request.goal.header.frame_id = "map";
  path.request.goal.pose.position.x = target->getX();
  path.request.goal.pose.position.y = target->getY();
  path.request.goal.pose.orientation.w = 1;
  //  cout << " (x_start, y_start) = (" << previous->getX() << "," <<
  //  previous->getY() << "), (x_goal, y_goal) = (" <<
  //          target->getX() << "," << target->getY() << ")" << endl;
  bool path_srv_call = path_client->call(path);
  if (path_srv_call) {
    // calculate path length
    path_len = getPathLen(path.response.plan.poses);
    //    if (path_len<1e3)
    //    {
    //      ROS_INFO("Path len is [%3.3f m.]",path_len);
    //    }
    //    else
    //    {
    //      ROS_INFO("Path len is infinite");
    //      path_len = 1000;
    //    }
  } else {
    ROS_INFO("[pure_navigation@updatePathMetrics] Path_finding Service call "
             "failed! ");
  }
  //  cout << "1: " << *travelledDistance << endl;
  // Update the distance counting
  *travelledDistance = *travelledDistance + path_len;
  //  cout << "2: " << *travelledDistance << endl;
  // Update the turning counting
  //  *numOfTurning = *numOfTurning + astar->getNumberOfTurning(path);
  // Update the scanning angle
  *totalAngle += scanAngle;
  // Update the number of configurations of the robot along the task
  (*numConfiguration)++;
  // Update counter of iterations
  (*count)++;
}

list<Pose> cleanHistory(vector<string> *history,
                        EvaluationRecords *record_history) {
  vector<string>::iterator it_history = history->begin();
  list<Pose> tmp_history;
  for (it_history; it_history != prev(history->end(), 1); it_history++) {
    if ((*it_history).back() == '1') {
      tmp_history.push_back(record_history->getPoseFromEncoding(*it_history));
    }
  }
  return tmp_history;
}

void printResult(long newSensedCells, long totalFreeCells, double precision,
                 long numConfiguration, double travelledDistance,
                 int numOfTurning, double totalAngle, double totalScanTime) {
  cout << "-----------------------------------------------------------------"
       << endl;
  cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells << endl;
  cout << "Total cell visited :" << numConfiguration << endl;
  cout << "Total travelled distance (meters): " << travelledDistance << endl;
  cout << "Total travel time: " << travelledDistance / 0.5 << "s, "
       << (travelledDistance / 0.5) / 60 << " m" << endl;
  cout << "I came back to the original position since i don't have any other "
          "candidate position"
       << endl;
  cout << "Total exploration time (s): " << travelledDistance / 0.5 << endl;
  cout << "Total number of turning: " << numOfTurning << endl;
  cout << "Sum of scan angles (radians): " << totalAngle << endl;
  cout << "Total time for scanning: " << totalScanTime << endl;
  cout << "Total time for exploration: "
       << travelledDistance / 0.5 + totalScanTime << "s, "
       << (travelledDistance / 0.5 + totalScanTime) / 60 << " m" << endl;
  if (newSensedCells < precision * totalFreeCells) {
    cout << "FINAL: MAP NOT EXPLORED! :(" << endl;
  } else {
    cout << "FINAL: MAP EXPLORED!" << endl;
  }

  cout << "-----------------------------------------------------------------"
       << endl;
}

Pose getCurrentPose(float resolution, float costresolution, dummy::Map *map,
                    double initFov, int initRange) {
  ros::Time _now_stamp_ = ros::Time(0);

  tf::StampedTransform start_pose_in_tf;
  tf::TransformListener _tf_listener;

  _tf_listener.waitForTransform("map", "base_link", _now_stamp_,
                                ros::Duration(2.0));
  try {
    _tf_listener.lookupTransform("map", "base_link", _now_stamp_,
                                 start_pose_in_tf);
  } catch (tf::TransformException &ex) {
    ROS_INFO("TRANSFORMS ARE COCKED-UP PAL! Why is that :=> %s", ex.what());
  }

  tf::Vector3 start_position = start_pose_in_tf.getOrigin();
  tf::Quaternion start_orientation = start_pose_in_tf.getRotation();

  geometry_msgs::PoseStamped start_pose;
  start_pose.header.stamp = start_pose_in_tf.stamp_;
  start_pose.header.frame_id = start_pose_in_tf.frame_id_;

  tf::pointTFToMsg(start_position, start_pose.pose.position);
  tf::quaternionTFToMsg(start_orientation, start_pose.pose.orientation);

  float initX = roundf(start_pose.pose.position.x * 100) / 100;
  float initY = roundf(start_pose.pose.position.y * 100) / 100;
  tf::Quaternion quat = tf::Quaternion(
      start_pose.pose.orientation.x, start_pose.pose.orientation.y,
      start_pose.pose.orientation.z, start_pose.pose.orientation.w);
  tfScalar angle = roundf(2 * atan2(quat[2], quat[3]) * 100) / 100;

  cout << endl
       << "Current position in the map frame:" << initX << "," << initY
       << " with orientation :" << angle << "(" << (angle * 180 / M_PI)
       << " deg)" << endl;

  int initOrientation = angle * 180 / M_PI;
  //  cout << "Orientation after casting: " << initOrientation << endl;

  // ATTENTION: should be adapted for cells different from 1mx1m
  // convert from map frame to image
  tf::Vector3 pose = tf::Vector3(initX, initY, 0.0);
  //  if (resolution >= 0 && resolution < 1 && resolution != costresolution)
  //  {
  //    //full resolution and scaling
  //    cout << "Full resolution and scaling" << endl;
  //    pose = pose / costresolution;
  ////    cout << "[BEFORE]Initial position in the Gazebo frame: " <<
  ///pose.getX() * costresolution<< "," << pose.getY() * costresolution << endl;
  //    Pose initialPose = Pose(map->getPathPlanningNumRows() - (long)
  //    pose.getY() + costorigin.position.y / costresolution, (long) pose.getX()
  //    - costorigin.position.x / costresolution,
  //                            initOrientation, initRange, initFov);
  //    return initialPose;
  //  }
  //  else
  //  {
  // 1mx1m
  //    cout << endl << "1mx1m" << endl;
  // cout << "[BEFORE]Initial position in the image frame: " << pose.getX()<<
  // "," << map.getPathPlanningNumRows() - (long)pose.getY() << endl;
  // NOTE: Y in map are X in image
  Pose initialPose = Pose(pose.getX(), pose.getY(), angle, initRange, initFov);
  return initialPose;
  //  }
}

void grid_callback(const nav_msgs::OccupancyGridConstPtr &msg) {
  // ROS_INFO("RECEIVED A MAP!");
  if (costmapReceived == 0) {
    ROS_INFO("CALLBACK FIRST!");
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

int getIndex(int x, int y) {
  int sx = costmap_grid.info.width;
  return y * sx + x;
}

void move(float x, float y, float orientation, float time_travel,
          list<Pose> *tabuList,
          std::list<std::pair<float, float> > *posToEsclude) {
  move_base_msgs::MoveBaseGoal goal;

  MoveBaseClient ac("move_base", true);
  // we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;

  double _orientation = orientation;
  // The orientation needs to be within [-M_PI, M_PI]
  if (orientation > M_PI)
    _orientation = orientation - 2 * M_PI;
  tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, _orientation);
  goal.target_pose.pose.orientation.x = q.getX();
  goal.target_pose.pose.orientation.z = q.getY();
  goal.target_pose.pose.orientation.z = q.getZ();
  goal.target_pose.pose.orientation.w = q.getW();

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ROS_INFO("[pure_navigation.cpp@move] Sending goal");
  //  cout << "   [pure_navigation.cpp@move] [map]goal: (" << x << "," << y <<
  //  ") with orientation: " << _orientation << "(" << _orientation * 180 / M_PI
  //  << ")"<< endl;
  ac.sendGoal(goal);

  time_travel = std::min(time_travel, (float)120.0);
  //  cout << "     [pure_navigation.cpp@move] Waiting for " << time_travel << "
  //  seconds" << endl;
  ac.waitForResult(ros::Duration(time_travel));

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("[pure_navigation.cpp@move] I'm moving...");
  } else {
    ROS_INFO("[pure_navigation.cpp@move] The base failed to move, adding this "
             "pose to the Tabulist and posToEsclude");
    std::pair<float, float> pairToRemove;
    pairToRemove = make_pair(x, y);
    posToEsclude->push_back(pairToRemove);
  }

  cout << endl;
}

double getPathLen(std::vector<geometry_msgs::PoseStamped> poses) {
  double len = 0;
  geometry_msgs::Point p1, p2;
  int npoints = poses.size();
  //  ROS_INFO("[pure_navigation.cpp@getPathLen]Path has [%d] points",npoints);
  if (npoints > 0) {
    for (int i = 1; i < npoints; i++) {
      p1 = poses[i].pose.position;
      p2 = poses[i - 1].pose.position;
      len += sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }
  } else {
    len = std::numeric_limits<double>::max();
    //        ROS_INFO("[pure_navigation.cpp@getPathLen]Empty path. Len set to
    //        infinite... ");
  }

  return len;
}

void showMarkerandNavigate(Pose target, ros::Publisher *marker_pub,
                           nav_msgs::GetPlan *path,
                           ros::ServiceClient *path_client,
                           list<Pose> *tabuList,
                           std::list<std::pair<float, float> > *posToEsclude) {
  //---------------------------PRINT GOAL POSITION
  geometry_msgs::PointStamped p;
  p.header.frame_id = "map";
  p.header.stamp = ros::Time::now();

  p.point.x = target.getX();
  p.point.y = target.getY();

  //  cout << "   New goal in map: X = " << p.point.x << ", Y = " << p.point.y
  //  << " m. "<<endl;
  marker_pub->publish(p);
  //----------------------------------------------
  move_base_msgs::MoveBaseGoal goal;

  // Update the destination point with the target
  path->request.goal.header.frame_id = "map";
  path->request.goal.pose.position.x = target.getX();
  path->request.goal.pose.position.y = target.getY();
  path->request.goal.pose.orientation.w = 1;
  //  cout << "[pure_navigation.cpp@showMarkerandNavigate] [PATH-START] = (" <<
  //  path->request.start.pose.position.x << ", " <<
  //  path->request.start.pose.position.y << ")" << endl;
  //  cout << "[pure_navigation.cpp@showMarkerandNavigate] [PATH-GOAL] = (" <<
  //  path->request.goal.pose.position.x << ", " <<
  //  path->request.goal.pose.position.y << ")" << endl;
  bool path_srv_call = path_client->call(*path);
  float path_len;
  if (path_srv_call) {
    // calculate path length
    path_len = getPathLen(path->response.plan.poses);
    if (path_len < 1e3) {
      //            ROS_INFO("[pure_navigation.cpp@showMarkerandNavigate] Path
      //            len is [%3.3f m.]", path_len);
    } else {
      //            ROS_INFO("[pure_navigation.cpp@showMarkerandNavigate] Path
      //            len is infinite");
      path_len = 1000;
    }
  } else {
    ROS_INFO(
        "[pure_navigation.cpp@showMarkerandNavigate] Service call failed! ");
    path_len = 1000;
  }

  float time_travel = 2 * path_len / min_robot_speed;
  //    cout << "[pure_navigation.cpp@showMarkerandNavigate] Target is at " <<
  //    path_len << " m from the robot" << endl;

  move(p.point.x, p.point.y, roundf(target.getOrientation() * 100) / 100,
       time_travel, tabuList,
       posToEsclude); // full resolution
}





void loadROSParams(){

  ros::NodeHandle private_node_handle("~");

  // LOAD ROS PARAMETERS ....................................
  private_node_handle.param("static_map_srv_name", static_map_srv_name, std::string("static_map"));
  private_node_handle.param("make_plan_srv_name", make_plan_srv_name, std::string("/move_base/make_plan"));
  private_node_handle.param("move_base_goal_topic_name", move_base_goal_topic_name, std::string("move_base_simple/goal"));
  private_node_handle.param("move_base_srv_name", move_base_srv_name, std::string("move_base"));
  private_node_handle.param("nav_grid_debug_topic_name", nav_grid_debug_topic_name, std::string("nav_grid_debug"));
  private_node_handle.param("planning_grid_debug_topic_name", planning_grid_debug_topic_name, std::string("planning_grid_debug"));
  private_node_handle.param("move_base_costmap_topic_name", move_base_costmap_topic_name, std::string("move_base/global_costmap/costmap"));
  private_node_handle.param("move_base_costmap_updates_topic_name", move_base_costmap_updates_topic_name, std::string("move_base/global_costmap/costmap_updates"));
  private_node_handle.param("marker_pub_topic_name", marker_pub_topic_name, std::string("goal_pt"));

}

void printROSParams(){
  ROS_INFO("/////////////////////////////////////////////////////////////////////////");
  ROS_INFO("[pure_navigation@printROSParams] Using the following ros params:");

  ROS_INFO("   - static_map_srv_name [%s]", static_map_srv_name.c_str());
  ROS_INFO("   - make_plan_srv_name [%s]", make_plan_srv_name.c_str());
  ROS_INFO("   - move_base_goal_topic_name [%s]", move_base_goal_topic_name.c_str());
  ROS_INFO("   - move_base_srv_name [%s]", move_base_srv_name.c_str());
  ROS_INFO("   - nav_grid_debug_topic_name [%s]", nav_grid_debug_topic_name.c_str());
  ROS_INFO("   - planning_grid_debug_topic_name [%s]", planning_grid_debug_topic_name.c_str());
  ROS_INFO("   - move_base_costmap_topic_name [%s]", move_base_costmap_topic_name.c_str());
  ROS_INFO("   - move_base_costmap_updates_topic_name [%s]", move_base_costmap_updates_topic_name.c_str());
  ROS_INFO("   - marker_pub_topic_name [%s]", marker_pub_topic_name.c_str());
  ROS_INFO("/////////////////////////////////////////////////////////////////////////");

  cout << "/////////////////////////////////////////////////////////////////////////" << endl;
  cout << "[pure_navigation@printROSParams] Using the following ros params:" << endl;
  cout << "   - static_map_srv_name [" << static_map_srv_name <<"]" << endl;
  cout << "   - make_plan_srv_name [" << make_plan_srv_name <<"]" << endl;
  cout << "   - move_base_goal_topic_name [" << move_base_goal_topic_name <<"]" << endl;
  cout << "   - move_base_srv_name [" << move_base_srv_name <<"]" << endl;
  cout << "   - nav_grid_debug_topic_name [" << nav_grid_debug_topic_name <<"]" << endl;
  cout << "   - planning_grid_debug_topic_name [" << planning_grid_debug_topic_name <<"]" << endl;
  cout << "   - move_base_costmap_topic_name [" << move_base_costmap_topic_name <<"]" << endl;
  cout << "   - move_base_costmap_updates_topic_name [" << move_base_costmap_updates_topic_name <<"]" << endl;
  cout << "   - marker_pub_topic_name [" << marker_pub_topic_name <<"]" << endl;
  cout << "/////////////////////////////////////////////////////////////////////////" << endl;

}


void createROSComms(){

  ros::NodeHandle nh;
  ros::Rate r(1);
  bool disConnected = true;

  // create service clients
  map_service_client_ = nh.serviceClient<nav_msgs::GetMap>(static_map_srv_name);
  path_client =   nh.serviceClient<nav_msgs::GetPlan>(make_plan_srv_name, true);

  // create publishers
  moveBasePub =   nh.advertise<geometry_msgs::PoseStamped>(move_base_goal_topic_name, 1000);
  gridPub = nh.advertise<grid_map_msgs::GridMap>(nav_grid_debug_topic_name, 1, true);
  planningPub = nh.advertise<grid_map_msgs::GridMap>(planning_grid_debug_topic_name, 1, true);
  marker_pub =  nh.advertise<geometry_msgs::PointStamped>(marker_pub_topic_name, 10);


  // create subscribers, only when we are sure the right people is publishing
  ROS_INFO("[pure_navigation@createROSComms] Waiting for move_base action server to come up");
  MoveBaseClient ac(move_base_srv_name, true);
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("[pure_navigation@createROSComms]... waiting ...");
  }


  while (disConnected) {
    ROS_INFO("[pure_navigation@createROSComms] Waiting for static_map service to respond...");
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
