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
void cleanPossibleDestination2(std::list<Pose> &possibleDestinations, Pose &p);
bool contains(std::list<Pose> &list, Pose &p);
bool containsPos(std::list<std::pair<float, float>> positionEscluded,
                 std::pair<float, float> p);
Pose createFromInitialPose(Pose pose, float variation, int range, int FOV);
// void gasDetection();
Pose getCurrentPose(float resolution, float costresolution, dummy::Map *map,
                    double initFov, int initRange);
int getIndex(int x, int y);
double getPathLen(std::vector<geometry_msgs::PoseStamped> poses);
double getPtuAngle(double mapAngle, int orientation);
void grid_callback(const nav_msgs::OccupancyGridConstPtr &msg);
void move(float x, float y, float orientation, float time_travel,
          list<Pose> *tabuList,
          std::list<std::pair<float, float>> *posToEsclude);
void printResult(long newSensedCells, long totalFreeCells, double precision,
                 long numConfiguration, double travelledDistance,
                 int numOfTurning, double totalAngle, double totalScanTime);
void pushInitialPositions(dummy::Map map, float x, float y, float orientation,
                          int range, int FOV, double threshold,
                          string actualPose,
                          vector<pair<string, list<Pose>>> *graph2,
                          ros::ServiceClient *path_client);
void showMarkerandNavigate(Pose target, ros::Publisher *marker_pub,
                           nav_msgs::GetPlan *path,
                           ros::ServiceClient *path_client,
                           list<Pose> *tabuList,
                           std::list<std::pair<float, float>> *posToEsclude);
// void stateCallback(const std_msgs::Int16::ConstPtr& sta);
void scanning();
void update_callback(const map_msgs::OccupancyGridUpdateConstPtr &msg);
void updatePathMetrics(
    int *count, Pose *target, Pose *previous, string actualPose,
    list<Pose> *nearCandidates, vector<pair<string, list<Pose>>> *graph2,
    dummy::Map *map, MCDMFunction *function, list<Pose> *tabuList,
    list<pair<float, float>> *posToEsclude, vector<string> *history,
    int encodedKeyValue, Astar *astar, long *numConfiguration,
    double *totalAngle, double *travelledDistance, int *numOfTurning,
    double scanAngle, ros::ServiceClient *path_client, bool backTracking);
double getPathLen(std::vector<geometry_msgs::PoseStamped> poses);

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
double min_robot_speed = 0.1;
nav_msgs::GetPlan path;

// Input :  15 180 0.95 0.12
// range centralAngle precision threshold
int main(int argc, char **argv) {

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

  auto startMCDM = chrono::high_resolution_clock::now();
  ros::init(argc, argv, "mcdm_exploration_framework_node");
  ros::NodeHandle nh;
  ros::ServiceClient map_service_client_ =
      nh.serviceClient<nav_msgs::GetMap>("static_map");
  ros::ServiceClient path_client =
      nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan", true);
  double path_len;
  bool path_srv_call;
  nav_msgs::GetMap srv_map;
  MoveBaseClient ac("move_base", true);
  ros::Publisher moveBasePub =
      nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
  ros::Publisher gridPub =
      nh.advertise<grid_map_msgs::GridMap>("nav_grid_debug", 1, true);
  ros::Publisher planningPub =
      nh.advertise<grid_map_msgs::GridMap>("planning_grid_debug", 1, true);
  //        ros::Subscriber ptu_sub =
  //        nh.subscribe("/ptu_control/state",10,stateCallback);
  ros::Subscriber costmap_sub;
  ros::Subscriber costmap_update_sub;
  ros::Rate r(10);

  ROS_INFO("Waiting for move_base action server to come up");
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("... waiting ...");
  }

  bool disConnected = true;
  while (disConnected) {
    ROS_INFO("Waiting for static_map service to respond...");
    if (map_service_client_.call(srv_map)) {
      costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>(
          "move_base/global_costmap/costmap", 100, grid_callback);
      costmap_update_sub = nh.subscribe<map_msgs::OccupancyGridUpdate>(
          "move_base/global_costmap/costmap_updates", 10, update_callback);
      disConnected = false;
    } else {
      r.sleep();
    }
  }

  cout << "Here" << endl;

  while (ros::ok()) {

    if (costmapReceived == 0) {
      ROS_INFO_STREAM_THROTTLE(60, "waiting for costmap" << std::endl);
      // cout << "Waiting for costmap" << endl;
    }

    if (costmapReceived == 1) {
      double initFov = atoi(argv[1]);
      initFov = initFov * M_PI / 180;
      FoV = initFov;
      int initRange = atoi(argv[2]);
      sensing_range = initRange;
      double precision = atof(argv[3]);
      double threshold = atof(argv[4]);

      /* resolution = 0 -> full resolution
       * resolution = 1 -> 1mx1m
       * resolution = X -> X%(full resolution)
       * NOTE: LOWER RES VALUE, HIGHER REAL RESOLUTION*/
      double resolution = atof(argv[5]);
      cout << "Config: " << endl;
      cout << "   InitFov: " << initFov << endl;
      cout << "   InitRange: " << initRange << endl;
      cout << "   precision: " << precision << endl;
      cout << "   threshold: " << threshold << endl;
      cout << "   Resolution: " << resolution
           << "\n   Costresolution: " << costresolution << endl;

      dummy::Map map = dummy::Map(resolution, costmap_grid);
      ROS_DEBUG("Map created correctly");
      map.plotPathPlanningGridColor("/tmp/pathplanning_start.png");
      map.plotGridColor("/tmp/nav_start.png");

      ros::Publisher marker_pub =
          nh.advertise<geometry_msgs::PointStamped>("goal_pt", 10);
      ROS_DEBUG("[pure_navigation.cpp@main] publisher created ...");
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
      vector<pair<string, list<Pose>>> graph2;
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
      history.push_back(function.getEncodedKey(target, 1));
      // amount of time the robot should do nothing for scanning the environment
      // ( final value expressed in second)
      unsigned int microseconds = 5 * 1000 * 1000;
      // cout << "total free cells in the main: " << totalFreeCells << endl;
      list<Pose> unexploredFrontiers;
      list<Pose> tabuList;
      std::list<std::pair<float, float>> posToEsclude;
      EvaluationRecords record;
      Astar astar;
      bool scan = true;
      double min_robot_speed = 0.1;
      nav_msgs::GetPlan path;

      while (sensedCells < precision * totalFreeCells - 200) {
        float x = target.getX();
        float y = target.getY();
        float orientation = roundf(target.getOrientation() * 100) / 100;
        ; // cast orientation in [0, 360]
        int range = target.getRange();
        double FOV = target.getFOV();
        string actualPose = function.getEncodedKey(target, 0);
        map.setCurrentPose(target);

        // string path =
        // astar.pathFind(target.getX(),target.getY(),previous.getX(),previous.getY(),map);
        // travelledDistance = travelledDistance + astar.lenghtPath(path);
        // numOfTurning = numOfTurning + astar.getNumberOfTurning(path);
        string encoding = to_string(target.getX()) + to_string(target.getY());
        visitedCell.emplace(encoding, 0);

        cout << "--------------------------------------------------------------"
                "---"
             << endl;
        cout << "Round : " << count + 1 << endl;
        cout << "Area sensed: " << newSensedCells << " / "
             << totalFreeCells - 200 << endl;
        target.setScanAngles(map.getSensingTime(x, y, orientation, FOV, range));
        totalAngle +=
            target.getScanAngles().second - target.getScanAngles().first;
        newSensedCells =
            sensedCells +
            map.performSensingOperation(x, y, orientation, FOV, range,
                                        target.getScanAngles().first,
                                        target.getScanAngles().second);
        map.updatePathPlanningGrid(x, y, range);
        gridPub.publish(map.toMessageGrid());
        planningPub.publish(map.toMessagePathPlanning());
        map.plotPathPlanningGridColor("/tmp/pathplanning_lastLoop.png");
        map.plotGridColor("/tmp/nav_lastLoop.png");
        map.findCandidatePositions(x, y, orientation, FOV, range);
        vector<pair<float, float>> candidatePosition =
            map.getCandidatePositions();
        map.emptyCandidatePositions();

        if (candidatePosition.size() == 0) {
          map.findCandidatePositions2(x, y, orientation, FOV, range);
          candidatePosition = map.getCandidatePositions();
          map.emptyCandidatePositions();
        }

        // if(scan) {
        //         //NOTE: perform gas sensing------------
        //         offsetY_base_rmld = 0.904;
        //         tilt_angle =
        //         (atan(sensing_range/offsetY_base_rmld)*(180/PI))-90;
        //         //tilt_angle = -10;
        //         num_pan_sweeps = 1;
        //         num_tilt_sweeps = 1;
        //         sample_delay = 0.1;
        //         //cout << "angolo finale:" << endl;
        //         max_pan_angle = getPtuAngle(target.getScanAngles().second,
        //         target.getOrientation());
        //         //cout << "angolo iniziale:" << endl;
        //         min_pan_angle = getPtuAngle(target.getScanAngles().first,
        //         target.getOrientation());
        //
        //         boost::thread mythread(scanning);
        //         mythread.join();
        //         min_pan_angle = 0;
        //         max_pan_angle = 0;
        //         //-------------------------------------
        // }

        //--------------------------------------------------
        /* Push in the graph the initial position with different orientations
         */

        if (count == 0) {
          string invertedPose = function.getEncodedKey(invertedInitial, 0);
          string eastPose = function.getEncodedKey(eastInitial, 0);
          string westPose = function.getEncodedKey(westInitial, 0);
          list<Pose> empty;
          std::pair<string, list<Pose>> pair1 = make_pair(invertedPose, empty);
          std::pair<string, list<Pose>> pair2 = make_pair(eastPose, empty);
          std::pair<string, list<Pose>> pair3 = make_pair(westPose, empty);
          graph2.push_back(pair1);
          graph2.push_back(pair2);
          graph2.push_back(pair3);
        }

        if (count != 0 &&
            (target.isEqual(invertedInitial) || target.isEqual(eastInitial) ||
             target.isEqual(westInitial))) {
          graph2.pop_back();
          actualPose = function.getEncodedKey(target, 0);
          pushInitialPositions(map, x, y, orientation, range, FOV, threshold,
                               actualPose, &graph2, &path_client);
        }
        //------------------------------------------------------

        if (candidatePosition.size() == 0) {

          // NOTE: TAKE THIS BRANCH IF THERE ARE NO CANDIDATE POSITION FROM THE
          // ACTUAL ONE

          cout << "No other candidate position" << endl;
          cout << "----- BACKTRACKING -----" << endl;

          if (graph2.size() > 0) {

            string targetString = graph2.at(graph2.size() - 1).first;
            target = record.getPoseFromEncoding(targetString);
            graph2.pop_back();
            history.push_back(function.getEncodedKey(target, 2));
            cout << "[BT]No significative position reachable. Come back to "
                    "previous position"
                 << endl;
            cout << "New target: x = " << target.getY()
                 << ",y = " << target.getX()
                 << ", orientation = " << target.getOrientation() << endl;
            count = count + 1;
            cout << "Graph dimension : " << graph2.size() << endl;
            scan = false;

          } else {
            cout << "----------------------------------------------------------"
                    "-------"
                 << endl;
            cout << "I came back to the original position since i don't have "
                    "any other candidate position"
                 << endl;
            cout << "Total cell visited :" << numConfiguration << endl;
            cout << "Total travelled distance (cells): " << travelledDistance
                 << endl;
            cout << "Total travel time: " << travelledDistance / 0.5 << endl;
            cout << "Total number of turning: " << numOfTurning << endl;
            cout << "Sum of scan angles (radians): " << totalAngle << endl;
            cout << "Total time for scanning: " << timeOfScanning << endl;
            cout << "FINAL: MAP NOT EXPLORED! :(" << endl;
            cout << "----------------------------------------------------------"
                    "-------"
                 << endl;
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
        } else {

          // NOTE: TAKE THIS BRANCH IF THERE ARE CANDIDATE POSITION FROM THE
          // ACTUAL ONE

          // need to convert from a <int,int pair> to a Pose with also
          // orientation,laser range and angle
          list<Pose> frontiers;
          vector<pair<float, float>>::iterator it = candidatePosition.begin();
          for (it; it != candidatePosition.end(); it++) {
            Pose p1 = Pose((*it).first, (*it).second, 0, range, FOV);
            Pose p2 = Pose((*it).first, (*it).second, 45, range, FOV);
            Pose p3 = Pose((*it).first, (*it).second, 90, range, FOV);
            Pose p4 = Pose((*it).first, (*it).second, 135, range, FOV);
            Pose p5 = Pose((*it).first, (*it).second, 180, range, FOV);
            Pose p6 = Pose((*it).first, (*it).second, 225, range, FOV);
            Pose p7 = Pose((*it).first, (*it).second, 270, range, FOV);
            Pose p8 = Pose((*it).first, (*it).second, 315, range, FOV);
            frontiers.push_back(p1);
            frontiers.push_back(p2);
            frontiers.push_back(p3);
            frontiers.push_back(p4);
            frontiers.push_back(p5);
            frontiers.push_back(p6);
            frontiers.push_back(p7);
            frontiers.push_back(p8);
          }

          unexploredFrontiers = frontiers;

          // cout << "Graph dimension : " << graph2.size() << endl;
          // cout << "Candidate position: " << candidatePosition.size() << endl;
          // cout <<"Frontiers: "<<  frontiers.size() << endl;
          EvaluationRecords *record = function.evaluateFrontiers(
              frontiers, &map, threshold, &path_client);
          // cout << "Record: " << record->size() << endl;
          // cout << "Evaluation Record obtained" << endl;

          if (record->size() != 0) {

            // NOTE: TAKE THIS BRANCH IF THERE ARE CANDIDATE POSITION THAT
            // SATISFY THE THRESHOLD

            // set the previous pose equal to the actual one(actually
            // represented by target)
            previous = target;
            std::pair<Pose, double> result = function.selectNewPose(record);
            target = result.first;
            targetPos = std::make_pair(target.getX(), target.getY());
            // if (contains(tabuList,target) == false) {
            if (!containsPos(posToEsclude, targetPos)) {

              // NOTE: TAKE THIS BRANCH IF THE TARGET ISN'T VISITED YET
              scan = true;
              count = count + 1;
              numConfiguration++;
              history.push_back(function.getEncodedKey(target, 1));
              cout << "Graph dimension : " << graph2.size() << endl;
              // tabuList.push_back(target);
              posToEsclude.push_back(
                  make_pair(roundf((target.getX() * 100) / 100),
                            roundf(target.getY() * 100 / 100)));
              std::pair<string, list<Pose>> pair =
                  make_pair(actualPose, frontiers);
              graph2.push_back(pair);

              showMarkerandNavigate(target, &marker_pub, &path, &path_client,
                                    &tabuList, &posToEsclude);

              // //---------------------------PRINT GOAL POSITION
              // geometry_msgs::PointStamped p;
              // p.header.frame_id = "map";
              // p.header.stamp = ros::Time::now();
              // //NOTE: as before, Y in map are X in image
              //
              //
              //
              // if(resolution >= 0 && resolution < 1 && resolution !=
              // costresolution) {
              //         //NOTE: full resolution
              //         p.point.x = (map.getNumGridRows() - target.getX() ) *
              //         costresolution;
              //         p.point.y = (target.getY() ) * costresolution;
              //
              // }else {
              //         //NOTE: 1mx1m
              //         p.point.x = (map.getPathPlanningNumRows() -
              //         target.getX() );//* costresolution;
              //         p.point.y = (target.getY() );// * costresolution;
              // }
              //
              // //cout << p.point.x << ","<< p.point.y << endl;
              //
              // tf::Vector3 vec =  tf::Vector3(p.point.x,p.point.y,0.0);
              //
              // //vec = transform.operator*(vec);
              //
              // p.point.x = vec.getY();
              // p.point.y = vec.getX();
              //
              // cout << "New goal in map: X = "<< p.point.x << ", Y = "<<
              // p.point.y << endl;
              //
              // //NOTE: not requested for testing purpose
              // //usleep(microseconds);
              // marker_pub.publish(p);
              // //----------------------------------------------
              //
              //
              // move_base_msgs::MoveBaseGoal goal;
              // double orientZ = (double)(target.getOrientation()* PI/(2*180));
              // double orientW =  (double)(target.getOrientation()* PI/(2 *
              // 180));
              // if(resolution != 0) {
              //         move(p.point.x + 0.5,p.point.y + 0.5, sin(orientZ),
              //         cos(orientW));
              // }else move(p.point.x,p.point.y, sin(orientZ), cos(orientW));
              scan = true;

            } else {

              // NOTE: TAKE THIS BRANCH IF THE TARGET IS ALREASY VISITED

              cout << "[BT - Tabulist]There are visible cells but the selected "
                      "one is already explored!Come back to previous position "
                      "in the graph"
                   << endl;
              cleanPossibleDestination2(graph2.at(graph2.size() - 1).second,
                                        target);
              string targetString = graph2.at(graph2.size() - 1).first;
              graph2.pop_back();
              target = record->getPoseFromEncoding(targetString);
              history.push_back(function.getEncodedKey(target, 2));
              cout << "New target: x = " << target.getX()
                   << ",y = " << target.getY()
                   << ", orientation = " << target.getOrientation() << endl;
              count = count + 1;
              cout << "Graph dimension : " << graph2.size() << endl;
              scan = false;
            }
          } else {
            // NOTE: TAKE THIS BRANCH IF THERE ARE NO CANDIDATE POSITIONS THAT
            // SATISFY THE THRESHOLD

            if (graph2.size() == 0)
              break;

            string targetString = graph2.at(graph2.size() - 1).first;
            target = record->getPoseFromEncoding(targetString);
            graph2.pop_back();

            if (!target.isEqual(previous)) {
              previous = target;
              cout << "[BT]Every frontier doen't satisfy the threshold. Come "
                      "back to previous position"
                   << endl;
              history.push_back(function.getEncodedKey(target, 2));
              cout << "New target: x = " << target.getX()
                   << ",y = " << target.getY()
                   << ", orientation = " << target.getOrientation() << endl;
              count = count + 1;
              cout << "Graph dimension : " << graph2.size() << endl;
              scan = false;
            } else {
              if (graph2.size() == 0) {
                cout << "No other possibilities to do backtracking on previous "
                        "positions since there are no more position in the "
                        "graph"
                     << endl;
                break;
              }
              string targetString = graph2.at(graph2.size() - 1).first;
              target = record->getPoseFromEncoding(targetString);
              graph2.pop_back();
              previous = target;
              cout << "[BT]There are no visible cells so come back to previous "
                      "position"
                   << endl;
              cout << "[BT]Cell already explored!Come back to previous position"
                   << endl;
              history.push_back(function.getEncodedKey(target, 2));
              cout << "New target: x = " << target.getX()
                   << ",y = " << target.getY()
                   << ", orientation = " << target.getOrientation() << endl;
              count = count + 1;
              scan = false;
            }
          }

          sensedCells = newSensedCells;
          sleep(2);
          frontiers.clear();
          candidatePosition.clear();
          delete record;
          // with rate
          ros::spinOnce();
          r.sleep();
          // without rate
          // ros::spin();
        }
      }

      map.drawVisitedCells();
      map.printVisitedCells(history);

      if (sensedCells >= precision * totalFreeCells - 200) {
        cout << "--------------------------------------------------------------"
                "---"
             << endl;
        cout << "Total cell visited :" << numConfiguration << endl;
        cout << "Total travelled distance (cells): " << travelledDistance
             << endl;
        cout << "Total travel time: " << travelledDistance / 0.5 << endl;
        cout << "Total number of turning: " << numOfTurning << endl;
        cout << "Sum of scan angles (radians): " << totalAngle << endl;
        cout << "Total time for scanning: " << timeOfScanning << endl;
        cout << "FINAL: MAP EXPLORED! :)" << endl;
        cout << "--------------------------------------------------------------"
                "---"
             << endl;
      } else {
        cout << "--------------------------------------------------------------"
                "---"
             << endl;
        cout << "Area sensed: " << newSensedCells << " / "
             << totalFreeCells - 200 << endl;
        cout << "I came back to the original position since i don't have any "
                "other candidate position"
             << endl;
        cout << "Total cell visited :" << numConfiguration << endl;
        cout << "Total travelled distance (cells): " << travelledDistance
             << endl;
        cout << "Total travel time: " << travelledDistance / 0.5 << endl;
        cout << "Total number of turning: " << numOfTurning << endl;
        cout << "Sum of scan angles (radians): " << totalAngle << endl;
        cout << "Total time for scanning: " << timeOfScanning << endl;
        cout << "FINAL: MAP NOT EXPLORED! :(" << endl;
        cout << "--------------------------------------------------------------"
                "---"
             << endl;
      }
      auto endMCDM = chrono::high_resolution_clock::now();

      double totalTimeMCDM =
          chrono::duration<double, milli>(endMCDM - startMCDM).count();
      cout << "Total time for MCDM algorithm : " << totalTimeMCDM << "ms, "
           << totalTimeMCDM / 1000 << " s, " << totalTimeMCDM / 60000 << " m "
           << endl;
      cout << "Total time in empirical way : "
           << travelledDistance / 0.25 + timeOfScanning / 1000 << endl;
      return 1;
    }

    // ROS_INFO_STREAM( "waiting for costmap" << std::endl);
    cout << "Spinning at the end" << endl;
    sleep(1);
    // with rate
    ros::spinOnce();
    r.sleep();
    // without rate
    // ros::spin();
  }
}

geometry_msgs::PoseStamped getCurrentPose() {
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

  return start_pose;
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

void move(int x, int y, double orZ, double orW) {
  move_base_msgs::MoveBaseGoal goal;

  MoveBaseClient ac("move_base", true);
  // we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.z = orZ;
  goal.target_pose.pose.orientation.w = orW;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("I'm moving...");
  else
    ROS_INFO("The base failed to move");
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

bool containsPos(std::list<std::pair<float, float>> positionEscluded,
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

void cleanPossibleDestination2(std::list<Pose> &possibleDestinations, Pose &p) {
  MCDMFunction function;
  // cout<<"I remove "<< function.getEncodedKey(p,0) << endl;
  // cout << possibleDestinations->size() << endl;

  std::list<Pose>::iterator findIter =
      std::find(possibleDestinations.begin(), possibleDestinations.end(), p);
  if (findIter != possibleDestinations.end()) {
    // cout << function.getEncodedKey(*findIter,0) << endl;
    possibleDestinations.erase(findIter);
  } else
    cout << "not found" << endl;
}

// void gasDetection(){
//
//        ros::NodeHandle n;
//        ros::ServiceClient client1 =
//        n.serviceClient<ptu_control::commandSweep>("/ptu_control/sweep");
//        ros::ServiceClient client2 =
//        n.serviceClient<amtec::GetStatus>("/amtec/get_status");
//
//        ptu_control::commandSweep srvSweep;
//
//        // Finding Tilt Angle:
//        //--------------------------
//        /*
//         *    | phi
//         *  0.904m |
//         *         |
//         *         |--
//         *         |_|_____________________________theta
//         *        sensing range
//         *  phi   = atan(sensing range / 0.904)
//         *  theta = atan(0.904 / sensing range)
//         *  tilt_angle = 90-phi
//         */
//
//        if(min_pan_angle > max_pan_angle) {
//                double tmp = min_pan_angle;
//                min_pan_angle = max_pan_angle;
//                max_pan_angle = tmp;
//        }
//
//        srvSweep.request.min_pan    = min_pan_angle;    //min_pan_angle;
//        //-10;
//        srvSweep.request.max_pan    = max_pan_angle;   //max_pan_angle;   //
//        10;
//        srvSweep.request.min_tilt   = tilt_angle;  //min_tilt_angle;  //-10;
//        srvSweep.request.max_tilt   = tilt_angle;  //max_tilt_angle;  //-10;
//        srvSweep.request.n_pan      = num_pan_sweeps;//  1;
//        srvSweep.request.n_tilt     = num_tilt_sweeps;//  1;
//        srvSweep.request.samp_delay = sample_delay; //0.1;
//
//        if (client1.call(srvSweep)) {
//                ROS_INFO("Gas detection in progress ...
//                <%.2f~%.2f,%.2f>",min_pan_angle,max_pan_angle,tilt_angle);
//        }else{
//                ROS_ERROR("Failed to initialize gas scanning.");
//        }
//
//
//}

// void stateCallback(const std_msgs::Int16::ConstPtr& sta){
//        //ROS_INFO("PTU status is...%d",sta->data);
//        statusPTU=sta->data;
//}

// void scanning(){
//        ros::NodeHandle nh("~");
//        ros::Subscriber ptu_sub;
//        ptu_sub =
//        nh.subscribe<std_msgs::Int16>("/ptu_control/state",100,stateCallback);
//        ros::AsyncSpinner spinner(0);
//        spinner.start();
//        auto start = chrono::high_resolution_clock::now();
////        gasDetection();
//        while(ros::ok()) {
//
//                while(statusPTU!=3) {
//                        sleep(1);
//                        //ROS_INFO("PTU status is...%d",statusPTU);
//                }
//                ROS_INFO("Scanning started!");
//                ros::WallDuration(5).sleep();
//                while(statusPTU!=0) {
//                        sleep(1);
//                        //ROS_INFO("PTU status is...%d",statusPTU);
//                }
//
//                ROS_INFO("Gas detection COMPLETED!");
//                auto end = chrono::high_resolution_clock::now();
//                double tmpScanning = chrono::duration<double,milli>(end
//                -start).count();
//                cout << "Time of current scan : "<< tmpScanning << " ms"
//                <<endl;
//                timeOfScanning = timeOfScanning + tmpScanning;
//                spinner.stop();
//                break;
//        }
//
//}

// double getPtuAngle(double mapAngle, int orientation)
//{
//        double ptuAngle = 0;
//        // get the angle in degrees
//
//
//        int tmp = mapAngle * 180 / PI;
//        cout << mapAngle << " -> " << tmp << endl;
//        cout << tmp <<endl;
//        /*
//           if(tmp >360){
//           ptuAngle = tmp - 360 - orientation;
//           }else ptuAngle = tmp - orientation;
//         */
//
//        if (tmp < 90) {
//                ptuAngle = tmp;
//        }else {
//                tmp = orientation + 360 - tmp;
//                if(tmp < 90) ptuAngle = tmp;
//                else ptuAngle = tmp - 360;
//        }
//
//        if(ptuAngle > 360) {
//                ptuAngle = ptuAngle - 360.0;
//        }
//        ptuAngle = ptuAngle * (-1);
//        //cout << ptuAngle <<endl;
//        return ptuAngle;
//}

void pushInitialPositions(dummy::Map map, float x, float y, float orientation,
                          int range, int FOV, double threshold,
                          string actualPose,
                          vector<pair<string, list<Pose>>> *graph2,
                          ros::ServiceClient *path_client) {
  NewRay ray;
  MCDMFunction function;
  map.findCandidatePositions(x, y, orientation, FOV, range);
  vector<pair<float, float>> candidatePosition = map.getCandidatePositions();
  map.emptyCandidatePositions();
  list<Pose> frontiers;
  vector<pair<float, float>>::iterator it = candidatePosition.begin();
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
  std::pair<string, list<Pose>> pair = make_pair(actualPose, nearCandidates);
  graph2->push_back(pair);
}

Pose createFromInitialPose(Pose pose, float variation, int range, int FOV) {
  Pose tmp = Pose(pose.getX(), pose.getY(), pose.getOrientation() + variation,
                  FOV, range);
  return tmp;
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

void showMarkerandNavigate(Pose target, ros::Publisher *marker_pub,
                           nav_msgs::GetPlan *path,
                           ros::ServiceClient *path_client,
                           list<Pose> *tabuList,
                           std::list<std::pair<float, float>> *posToEsclude) {
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

void move(float x, float y, float orientation, float time_travel,
          list<Pose> *tabuList,
          std::list<std::pair<float, float>> *posToEsclude) {
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