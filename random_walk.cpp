#include "Criteria/traveldistancecriterion.h"
#include "PathFinding/astar.h"
#include "map.h"
#include "mcdmfunction.h"
#include "newray.h"
#include "utils.h"
#include "radio_models/propagationModel.cpp"
#include <algorithm>
#include <iostream>
#include <iterator>
#include <random>

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
// mfc: we will record using stats_pub
//#include "record_ros/record.h"
//#include "record_ros/String_cmd.h"
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
bool use_mcdm = true;

double batteryTime = MAX_BATTERY;
double batteryPercentage = 100;

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
Utilities utils;

// Ros services/subscribers/publishers
ros::ServiceClient map_service_client_;
ros::ServiceClient path_client;
// mfc: we will record using stats_pub
//ros::ServiceClient rosbag_client;
nav_msgs::GetMap srv_map;
ros::Publisher moveBasePub;
ros::Subscriber costmap_sub;
ros::Subscriber costmap_update_sub;
ros::Publisher gridPub;
ros::Publisher planningPub;
ros::Publisher marker_pub;
// mfc: we will record using stats_pub
//record_ros::String_cmd srv_rosbag;

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
  ros::init(argc, argv, "next_best_sense_node");

  //mfc Load params from ros
  loadROSParams();

  //mfc Load params from ros
  printROSParams();

  // create ROS connections/services
  createROSComms();
  double path_len;
  bool path_srv_call;
  ros::Rate r(1);

// mfc: we will record using stats_pub
//   // Start recording the bag
//   srv_rosbag.request.cmd = "record";
//   if (rosbag_client.call(srv_rosbag)){
//     cout << "Start recording the bag..." << endl;
//     sleep(5);
//   }else{
//     cout << "Error occurring while recording the bag. Exiting now!" << endl;
// //    exit(0);
//   }


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
      double w_info_gain = 0.33;//atof(argv[6]);
      double w_travel_distance = 0.33;//atof(argv[7]);
      double w_sensing_time = 0.3;//atof(argv[8]);
      std::string out_log = "/home/pulver/Desktop/MCDM/random_walk/gazebo_inb3123_v2.csv";
      std::string coverage_log = "/home/pulver/Desktop/MCDM/random_walk/gazebo_inb3123_coverage_v2.csv";
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
          utils.getCurrentPose(resolution, costresolution, &map, initFov, initRange);
      Pose target = start_pose;
      Pose previous = target;
      Pose invertedInitial = utils.createFromInitialPose(start_pose, M_PI, initRange, initFov);
      Pose eastInitial = utils.createFromInitialPose(start_pose, M_PI / 2, initRange, initFov);
      Pose westInitial = utils.createFromInitialPose(start_pose, 3 * M_PI / 2, initRange, initFov);
      std::pair<float, float> targetPos;
      long numConfiguration = 1;
      vector<pair<string, list<Pose> >> graph2;
      bool backTracking = false;
      NewRay ray;
      ray.setGridToPathGridScale(gridToPathGridScale);
      MCDMFunction function(w_info_gain, w_travel_distance, w_sensing_time, use_mcdm);
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

      auto startMCDM = ros::Time::now().toSec();
      long n_obsts, n_free, n_vist, n_others;
      map.countPathPlanningCells( &n_obsts, &n_free, &n_vist, &n_others);
      cout << "Number of free cells in PlanningGrid: " << n_free << endl;
      grid_map::Position next_random_position;
      std::random_device rd; // obtain a random number from hardware
      std::mt19937 eng(rd()); // seed the generator
      std::uniform_int_distribution<> distr(0, 359); // define the range
      float x;
      float y;
      float orientation;
      int range;
      double FOV;
      string content;
      float tag_coverage_percentage = 0.0;

      do {
//
          target = utils.getCurrentPose(resolution, costresolution, &map, initFov, initRange);
          cout << "\n============================================" << endl;
          cout << "New iteration, position: " << target.getX() << "," << target.getY() <<
            "[ "<< newSensedCells << " sensed] - [" << totalFreeCells << " total]" <<
            "[ "<< 100 * float(newSensedCells)/float(totalFreeCells) << " %] - [" <<
            (ros::Time::now().toSec() - startMCDM ) / 60.0 << " min ]" << endl;

          // Look for the RFID coverage value
          std_msgs::Float32ConstPtr msg = ros::topic::waitForMessage<std_msgs::Float32>("/tag_coverage", ros::Duration(1));
          if(msg != NULL){
            tag_coverage_percentage = msg->data;
          }

          content = to_string(numConfiguration) 
                    + "," + to_string(100 * float(newSensedCells)/float(totalFreeCells)) 
                    + "," + to_string(tag_coverage_percentage) 
                    + "," + to_string(travelledDistance) + "\n" ;
          utils.saveCoverage(coverage_log, content, true );

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
          // Perform a scanning operation
          newSensedCells = sensedCells +  map.performSensingOperation(x, y, orientation, FOV, range,
                                          target.getScanAngles().first,
                                          target.getScanAngles().second);
          // Calculate the scanning angle
          double scanAngle = target.getScanAngles().second - target.getScanAngles().first;
          // Update the overall scanning time
          totalScanTime += utils.calculateScanTime(scanAngle * 180 / M_PI);
                    map.getPathPlanningIndex(x, y, cell_i, cell_j);
          map.updatePathPlanningGrid(x, y, range);
          gridPub.publish(map.toMessageGrid());
          planningPub.publish(map.toMessagePathPlanning());
          map.plotPathPlanningGridColor("/tmp/pathplanning_lastLoop.png");
          map.plotGridColor("/tmp/nav_lastLoop.png");

          // If the exploration just started
          next_random_position = map.getRandomFreeCellPosition();
          cout << "Next position: " << next_random_position.x() << ", " << next_random_position.y() << endl;
          orientation = distr(eng);  // get a random orientation between 0 and 359
          orientation = orientation * M_PI / 180.0;
          target.updateFromGridMapPosition(next_random_position, orientation, range, FOV);
          cout << "Target: " << target.getX() << ", " << target.getY() << endl;
          success = utils.showMarkerandNavigate(target, &marker_pub, &path, &path_client,
                                                    &tabuList, &posToEsclude, min_robot_speed, robot_radius, &batteryTime);
          if (success == true){
            utils.updatePathMetrics(
                &count, &target, &previous, actualPose, &nearCandidates,
                &graph2, &map, &function, &tabuList, &posToEsclude,
                &history, encodedKeyValue, &numConfiguration,
                &totalAngle, &travelledDistance, &numOfTurning, scanAngle,
                &path_client, backTracking, robot_radius);
          }
          sensedCells = newSensedCells;


      }
      // Perform exploration until a certain coverage is achieved
      while (sensedCells < precision * totalFreeCells);
      // Plotting utilities
      map.drawVisitedCells();
      map.printVisitedCells(history);

      cout << "Num configuration: " << numConfiguration << endl;
      cout << "Travelled distance calculated during the algorithm: "
           << travelledDistance << endl;

      cout << "------------------ HISTORY -----------------" << endl;
      // Calculate which cells have been visited only once
      list<Pose> tmp_history = utils.cleanHistory(&history, &record);
      utils.calculateDistance(tmp_history, &path_client, robot_radius);

      cout << "------------------ TABULIST -----------------" << endl;
      utils.calculateDistance(tabuList, &path_client, robot_radius);

      // Trasform distance in meters
      if (resolution ==
          1.0) // Corridor map has a resolution of 0.5 meter per cell
      {
        travelledDistance = travelledDistance / 2;
      }

      utils.printResult(newSensedCells, totalFreeCells, precision,
                            numConfiguration, travelledDistance, numOfTurning,
                            totalAngle, totalScanTime, resolution,
                            w_info_gain, w_travel_distance, w_sensing_time, out_log);

      cout << "-----------------------------------------------------------------" << endl;
      auto endMCDM = ros::Time::now().toSec();;

      double totalTimeMCDM = endMCDM - startMCDM;
      cout << "Total time for MCDM algorithm : " << totalTimeMCDM << "s, "
           << totalTimeMCDM / 60 << " m " << endl;
      cout << "Spinning at the end" << endl;

      // mfc: we will record using stats_pub
      // // Stop recording the bag
      // srv_rosbag.request.cmd = "stop";
      // if (rosbag_client.call(srv_rosbag)){
      //   cout << "Stop recording the bag..." << endl;
      //   sleep(5);
      // }else{
      //   cout << "Error occurring while stop recording the bag. Exiting now!" << endl;
      // }

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
  // mfc: we will record using stats_pub
  //rosbag_client = nh.serviceClient<record_ros::String_cmd>(rosbag_srv_name);

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
