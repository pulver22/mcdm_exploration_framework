#include "Criteria/criterion.h"
#include "PathFinding/astar.h"
#include "map.h"
#include "mcdmfunction.h"
#include "newray.h"
#include "utils.h"
#include "constants.h"
#include "radio_models/propagationModel.cpp"
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>  // std::accumulate

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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// mfc ...
#include <ros/console.h>
// mfc: we will record using stats_pub
// #include "record_ros/record.h"
// #include "record_ros/String_cmd.h"
// mfc ...
#include "rfid_grid_map/GetBeliefMaps.h"
#include "rfid_grid_map/GetFakeBeliefMaps.h"
#include "strands_navigation_msgs/TopologicalMap.h"
#include "strands_navigation_msgs/GetRouteTo.h"
#include "strands_navigation_msgs/GetRouteBetween.h"
#include "bayesian_topological_localisation/LocaliseAgent.h"
#include "bayesian_topological_localisation/DistributionStamped.h"
#include "bayesian_topological_localisation/UpdateLikelihoodObservation.h"
// #include "bayesian_topological_localisation/UpdatePriorLikelihoodObservation.h"
// #include "bayesian_topological_localisation/Predict.h"
#include <visualization_msgs/Marker.h>
#include <ctime>
#include <iomanip>
#include <experimental/filesystem> // or #include <filesystem> for C++17 and up
// #include "rasberry_people_perception/NoisyGPS.h"
#include "ncnr_logging/GetRadiationPredictions.h"
#include "nav_msgs/OccupancyGrid.h"
#include "grid_map_msgs/GridMap.h"
#include "ncnr_logging/GetRadiationLevel.h"
#include "gp_node/SetQueryPoints.h"
#include "gp_node/AddReading.h"
#include "gp_node/GetPredictions.h"
    
    
using namespace std;
using namespace dummy;

namespace fs = std::experimental::filesystem;

// ROS varies
bool move(float x, float y, float orientation, float time_travel,
          list<Pose> *tabuList,
          std::list<std::pair<float, float> > *posToEsclude);
void update_callback(const map_msgs::OccupancyGridUpdateConstPtr &msg);
void grid_callback(const nav_msgs::OccupancyGridConstPtr &msg);
void belief_topomap_callback(const bayesian_topological_localisation::DistributionStampedConstPtr &msg);
void topological_map_callback(const strands_navigation_msgs::TopologicalMapConstPtr &msg);
void printROSParams();
void loadROSParams();
ros::NodeHandle createROSComms();
void tag_coverage_callback(const std_msgs::Float32 msg);
// void radiation_map_callback(const grid_map_msgs::GridMap msg);
void gps_callback(const visualization_msgs::MarkerConstPtr &msg);
void sensing();



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>MoveBaseClient;
vector<int> occdata;
int costmapReceived = 0;
int topoMapReceived = 0;
float costresolution;
int costwidth;
int costheight;
geometry_msgs::Pose costorigin;
nav_msgs::OccupancyGrid costmap_grid, mean_occ_grid, var_occ_grid;
strands_navigation_msgs::TopologicalMap topological_map;
float current_rad_level = 0;

double min_pan_angle, max_pan_angle, min_tilt_angle, max_tilt_angle,
    sample_delay, tilt_angle;
int num_pan_sweeps, num_tilt_sweeps;
double sensing_range, offsetY_base_rmld, FoV;
int statusPTU, prevStatusPTU;
double timeOfScanning = 0;
bool btMode = false;
// nav_msgs::GetPlan path;
float tag_coverage_percentage = 0.0;
GridMap radiation_mean_map, radiation_var_map;
vector<bayesian_topological_localisation::DistributionStamped> belief_topomaps;
grid_map_msgs::GridMap radiation_map_msg;
GridMapRosConverter converter;
int belief_counter = 0;


//  ROS PARAMETERS ....................................
std::string static_map_srv_name, make_plan_srv_name, make_topo_plan_srv_name, get_topo_distances_srv_name;
std::string radiation_map_srv_name, fake_radiation_map_srv_name, radiation_level_srv_name, set_query_pts_srv_name, add_reading_srv_name, get_predictions_srv_name;
std::string move_base_goal_topic_name, move_base_srv_name, move_base_costmap_topic_name, move_base_local_costmap_topic_name, move_base_costmap_updates_topic_name;
std::string nav_grid_debug_topic_name, planning_grid_debug_topic_name;
std::string marker_pub_topic_name;
std::string rosbag_srv_name;
std::string gazebo_model_state_srv_name;
std::string experiment_finished_topic_name;
double robot_radius;
Utilities utils;
std::string stats_topic_name;
std::string topological_map_topic_name;
std::string localization_srv_name;
std::string pf_srv_name, pf_stateless_srv_name;
// std::string gps_srv_name;
std::string pf_topic_name;


// Ros services/subscribers/publishers
ros::ServiceClient map_service_client_, path_client,topo_path_client, topo_distances_client;
ros::ServiceClient radiation_map_client, fake_radiation_map_client, radiation_level_client, set_query_pts_client, add_readings_client, get_predictions_client;
ros::ServiceClient localization_client, pf_client, pf_stateless_client, gps_client;
ros::ServiceClient gazebo_model_state_client;
pair<bool, bayesian_topological_localisation::DistributionStamped> tmp_belief_topo;
vector<ros::ServiceClient> pf_likelihoodClient_list, pf_stateless_likelihoodClient_list, gps_client_list;
vector<bayesian_topological_localisation::DistributionStamped> stateless_belief_history;
vector<unordered_map<float, std::pair<string, bayesian_topological_localisation::DistributionStamped>>> mapping_time_belief;
prediction_tools prediction_tools;
// mfc: we will record using stats_pub
//ros::ServiceClient rosbag_client;
nav_msgs::GetMap srv_map;
grid_map_msgs::GridMap srv_radiation_map;
ros::Publisher moveBasePub;
ros::Subscriber costmap_sub;
ros::Subscriber costmap_update_sub;
ros::Subscriber tag_coverage_sub;
ros::Subscriber radiation_map_sub;
ros::Subscriber topo_map_sub;
ros::Subscriber pf_sub;
ros::Subscriber gps_sub;
std::vector<ros::Subscriber> pf_topoMap_sub_list;
ros::Publisher gridPub;
ros::Publisher planningPub;
ros::Publisher marker_pub;
ros::Publisher pf_pub;
ros::Publisher rad_mean_grid_pub, rad_var_grid_pub;
vector<ros::Publisher> pf_topoMap_pub_list;
// vector<string> tag_discovered;

// mfc: we will record using stats_pub
// record_ros::String_cmd srv_rosbag;
ros::Publisher stats_pub;
ros::Publisher experiment_finished_pub;

vector<string> current_tag_waypoint_prediction;
geometry_msgs::Pose pf_tag_pose, gt_tag_pose, gps_tag_pose;
double distance_pf_gt;
grid_map::GridMap rad_grid_map;

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

  // sets console output to debug mode...
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //                                    ros::console::levels::Debug)) {
  //   ros::console::notifyLoggerLevelsChanged();
  // }
  // mfc ...........................
  ros::init(argc, argv, "next_best_sense_node");

  // mfc Load params from ros
  loadROSParams();

  // mfc Load params from ros
  printROSParams();

  // create ROS connections/services
  ros::NodeHandle nh = createROSComms();
  double path_len;
  bool path_srv_call;
  ros::Rate r(20);

  std_msgs::String stats_msg;
  std::stringstream stats_buffer;
  double coverage;

  // Create srv request for the RFID belief map
  // nav_msgs::GetMap radiation_map_srv;
  ncnr_logging::GetRadiationPredictions radiation_map_srv;
  ncnr_logging::GetRadiationLevel radiation_level_srv;
  bayesian_topological_localisation::LocaliseAgent localization_srv;
  bayesian_topological_localisation::UpdateLikelihoodObservation prediction_srv;
  // bayesian_topological_localisation::Predict prediction_stateless_srv;
  // rasberry_people_perception::NoisyGPS gps_srv;


  // first time, add header. THIS SHOULD MATCH WHAT YOU PUBLISH LATER!!!!!!
  stats_buffer.str("coveragePercent, numConfiguration, backTracking");
  stats_msg.data = stats_buffer.str();
  stats_pub.publish(stats_msg);

  // Start recording the bag
  // srv_rosbag.request.cmd = "record";
  // if (rosbag_client.call(srv_rosbag)){
  //   cout << "Start recording the bag..." << endl;
  //   sleep(5);
  // }else{
  //   cout << "Error occurring while recording the bag. Exiting now!" << endl;
  //   ros::shutdown();
  // }

  std_msgs::Bool finished;
  finished.data = false;
  experiment_finished_pub.publish(finished);

  while (ros::ok()) {

    if (costmapReceived == 0) {
      ROS_INFO_STREAM_THROTTLE(60, "waiting for costmap" << std::endl);
      // cout << "Waiting for costmap" << endl;
    }
    if (topoMapReceived == 0) {
      ROS_INFO_STREAM_THROTTLE(60, "waiting for topologicalmap" << std::endl);
      // cout << "Waiting for costmap" << endl;
    }
    // boost::thread mythread(sensing); // Starting listening to RFID
    if (costmapReceived == 1 and topoMapReceived == 1) {
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
      double w_info_gain = atof(argv[6]);
      double w_travel_distance = atof(argv[7]);
      double w_rad_mean = atof(argv[8]);
      double w_rad_variance = atof(argv[10]);
      double w_battery_status = atof(argv[9]);
      // Normalize the weight such that their sum is always equal to 1
      double norm_w_info_gain, norm_w_travel_distance, norm_w_rad_mean,
      norm_w_rad_variance, norm_w_battery_status;
      double sum_w = w_info_gain + w_travel_distance + w_rad_mean +
                    w_rad_variance + w_battery_status;
      norm_w_info_gain = w_info_gain / sum_w;
      norm_w_travel_distance = w_travel_distance / sum_w;
      norm_w_rad_mean = w_rad_mean / sum_w;
      norm_w_battery_status = w_battery_status / sum_w;
      norm_w_rad_variance = w_rad_variance / sum_w;
      bool use_mcdm = bool(atoi(argv[11]));
      int num_tags = atoi(argv[12]);
      int max_iterations = atoi(argv[13]);
      float gp_var_threshold = atof(argv[14]);
      std::string path_distances_map = (argv[15]);
      std::string log_dest_folder = (argv[16]);
      auto t = std::time(nullptr);
      auto tm = *std::localtime(&t);
      std::ostringstream oss;
      oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S/");
      auto str = oss.str();
      string map_path = log_dest_folder + path_distances_map ;
      std::vector<std::string> pf_log, gt_log, gps_log, pf_vs_gt_log;
      std::string coverage_log, out_log, rad_readings_log;
      coverage_log = log_dest_folder + "nbs_coverage.csv";
      out_log = log_dest_folder + "nbs_result.csv";
      rad_readings_log = log_dest_folder + "readings.csv";
      cout << "Config: " << endl;
      cout << "   InitFov: " << initFov << endl;
      cout << "   InitRange: " << initRange << endl;
      cout << "   precision: " << precision << endl;
      cout << "   threshold: " << threshold << endl;

      cout << "   Resolution: " << resolution
           << "\n   Costresolution: " << costresolution << endl;
      cout << "Criteria: " << endl;
      cout << "   w_info_gain: " << norm_w_info_gain << endl;
      cout << "   w_travel_distance: " << norm_w_travel_distance << endl;
      cout << "   w_rad_mean: " << norm_w_rad_mean << endl;
      cout << "   w_battery_status: " << norm_w_battery_status << endl;
      cout << "   w_rad_variance: " << norm_w_rad_variance << endl;

      // dummy::Map map = dummy::Map(costresolution, costresolution, costwidth,
      // costheight, occdata, costorigin);
      dummy::Map map = dummy::Map(resolution, costmap_grid);
      ROS_DEBUG("Map created correctly");
      list<Pose> topoMap;
      unordered_map<string, string> mappingWaypoints;
      utils.convertStrandTopoMapToListPose(
          &topological_map, &topoMap, initRange, initFov, &mappingWaypoints);
      prediction_tools.topoMap = topoMap;
      for (auto it = topoMap.begin(); it != topoMap.end(); it++){
        prediction_tools.coordinates.push_back(make_pair(it->getX(), it->getY()));
        prediction_tools.x_values.push_back(it->getX());
        prediction_tools.y_values.push_back(it->getY());
      }
      ROS_DEBUG("TopologicalMap created");

      // ORI: Set the topo-nodes coordinates to the GP
      gp_node::SetQueryPoints gp_set_points;
      // double arr[prediction_tools.x_values.size()];
      // std::copy(prediction_tools.x_values.begin(), prediction_tools.x_values.end(), arr);
      // gp_set_points.request.x_values = arr;
      // std::copy(prediction_tools.y_values.begin(), prediction_tools.y_values.end(), arr);
      // gp_set_points.request.y_values = arr;
      gp_set_points.request.x_values = prediction_tools.x_values;
      gp_set_points.request.y_values = prediction_tools.y_values;
      bool gp_query_srv_call = set_query_pts_client.call(gp_set_points);
      if(gp_query_srv_call){
        cout << "[ORI] GP points correctly set!" << endl;
      }else {
        cout << "[ORI] ERROR: It was not possible to correctly set the coordinates for the GP." << endl;
      }

      // NOTE: let's create a map to store distance between each node and every other node;
      double start = ros::Time::now().toSec();
      std::unordered_map<string, double> distances_map;
      cout << "Loading distances matrix from disk ..." << endl;
      bool successfull_loading = utils.loadMap(&distances_map, map_path);
      if (successfull_loading == true){
        cout << "   Distances map contains: " << distances_map.size() << "entries" << endl;
      }else{
        cout << "It doesn't exist. Create a new one ..." << endl;
        strands_navigation_msgs::GetRouteBetween route;
        double distance;
        for(int i = 0; i < topological_map.nodes.size(); i++){
          for(int j = 0; j <= i; j++){
            route.request.origin = topological_map.nodes.at(i).name;
            route.request.goal = topological_map.nodes.at(j).name;
            bool route_srv_call = topo_distances_client.call(route);
            if(route_srv_call){
              cout << "   Called!" << endl;
              distance = route.response.route.source.size();
            }else {
              distance = 1000;
            }
            if (i == j) distance = 0;
            distances_map.emplace(topological_map.nodes.at(i).name + topological_map.nodes.at(j).name, distance);
            distances_map.emplace(topological_map.nodes.at(j).name + topological_map.nodes.at(i).name, distance);
          }
          
        }
        cout << "    Completed! [" << ros::Time::now().toSec() - start << "s]" << endl;
        utils.saveMap(&distances_map, map_path);
        cout << "Saving on disk completed" << endl;
      }

      map.plotPathPlanningGridColor("/tmp/pathplanning_start.png");

      map.plotGridColor("/tmp/nav_start.png");

      int gridToPathGridScale = map.getGridToPathGridScale();

      // Get the initial pose in map frame
      Pose start_pose = utils.getCurrentPose(resolution, costresolution, &map,
                                             initFov, initRange);
      Pose target = start_pose;
      Pose previous = target;
      Pose invertedInitial =
          utils.createFromInitialPose(start_pose, M_PI, initRange, initFov);
      Pose eastInitial =
          utils.createFromInitialPose(start_pose, M_PI / 2, initRange, initFov);
      Pose westInitial = utils.createFromInitialPose(start_pose, 3 * M_PI / 2,
                                                     initRange, initFov);
      std::pair<float, float> targetPos;
      long numConfiguration = 1;
      vector<pair<string, list<Pose>>> graph2;
      bool backTracking = false;
      NewRay ray;
      ray.setGridToPathGridScale(gridToPathGridScale);
      MCDMFunction function(norm_w_info_gain, norm_w_travel_distance, norm_w_rad_mean,
                            norm_w_rad_variance, norm_w_battery_status, use_mcdm);
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
      std::list<std::pair<float, float>> posToEsclude;
      EvaluationRecords record;
      bool scan = true;
      double totalScanTime = 0;
      int encodedKeyValue = 0;

      double batteryTime = MAX_BATTERY;
      double batteryPercentage = 100;

      // RFID
      double absTagX = 0;  // std::stod(argv[12]); // m.
      double absTagY = 0;  // std::stod(argv[11]); // m.
      double freq = 0;     // std::stod(argv[13]); // Hertzs
      double txtPower = 0; // std::stod(argv[14]); // dBs
      double rxPower = 0;
      // std::pair<int, int> relTagCoord;
      long i, j;
      long cell_i, cell_j;
      double targetX_meter, targetY_meter;
      bool success = false;

      auto startMCDM = ros::Time::now().toSec();
      string content;

      double entropy_map = 0;
      bool emptyTabuList = false;
      int tabuListCount = MAX_TABULIST_COUNT;
      bool explorationCompleted = false;

      bool break_loop = false;
      string robotName = "jackal";

      

      belief_topomaps.push_back(bayesian_topological_localisation::DistributionStamped());
      // sleep(5.0);

      // add the service client to utils so it can use it
      utils.setGazeboModelStateClient(gazebo_model_state_client);

      // ORI utils
      float initial_map_variance, current_map_variance = 0.0;
      float variance_ratio;

      do {

        record.clear();
        // if (btMode == false) {
        while (record.size() == 0) {
          // At every iteration, the current pose of the robot is taken from the
          // TF-tree
          target = utils.getCurrentPose(resolution, costresolution, &map,
                                        initFov, initRange);
          cout << "\n============================================" << endl;
          cout << "New iteration[" << count + 1 
               << "], position: " << target.getX() << "," << target.getY() 
               << "[ " << newSensedCells << " sensed] - ["
               << totalFreeCells << " total]"
               << "[ " << coverage << " %] - ["
               << (ros::Time::now().toSec() - startMCDM) / 60.0 << " min ] - ["
               << batteryPercentage << "% battery left]" << endl;

          // Look for the RFID coverage value
          std_msgs::Float32ConstPtr msg =
              ros::topic::waitForMessage<std_msgs::Float32>("/tag_coverage",
                                                            ros::Duration(1));
          if (msg != NULL) {
            tag_coverage_percentage = msg->data;
          }

          content =
              to_string(numConfiguration) + "," +
              to_string(100 * float(newSensedCells) / float(totalFreeCells)) +
              "," + to_string(tag_coverage_percentage) + "," +
              to_string(travelledDistance) + "\n";
          // cout << tag_coverage_percentage << endl;
          utils.filePutContents(coverage_log, content, true);
          ROS_DEBUG("  ==> Saving the coverage log ...");

          map.getPathPlanningIndex(target.getX(), target.getY(), i, j);
          map.getPathPlanningPosition(targetX_meter, targetY_meter, i, j);
          map.getGridIndex(target.getX(), target.getY(), i, j);
          gridPub.publish(map.toMessageGrid());

          // Update starting point in the path
          // path.request.start.header.frame_id = "map";
          // path.request.start.pose.position.x = target.getX();
          // path.request.start.pose.position.y = target.getY();
          // path.request.start.pose.orientation.w = 1;

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
          double X_max = range;
          // double X_min = 1;
          double X_min = X_max;  // scanning a circle
          double focal_length = (X_max - X_min) / 2.0; // (X_max - X_min)/2
          double major_axis = focal_length + X_min;    // (focal_length + X_min)
          double minor_axis = sqrt(pow(major_axis, 2) - pow(focal_length, 2));
          newSensedCells = sensedCells + map.performSensingOperationEllipse(
                                             x, y, orientation, FOV, range,
                                             target.getScanAngles().first,
                                             target.getScanAngles().second,
                                             minor_axis, major_axis);
          // Calculate the scanning angle
          double scanAngle =
              target.getScanAngles().second - target.getScanAngles().first;
          // Update the overall scanning time
          totalScanTime += utils.calculateScanTime(scanAngle * 180 / M_PI);
          map.getPathPlanningIndex(x, y, cell_i, cell_j);
          map.updatePathPlanningGrid(x, y, range);

          gridPub.publish(map.toMessageGrid());
          planningPub.publish(map.toMessagePathPlanning());
          map.plotPathPlanningGridColor("/tmp/pathplanning_lastLoop.png");
          map.plotGridColor("/tmp/nav_lastLoop.png");
          
          // here save the tag's closest nodes
          std::vector<string> closest_waypoints;
          // here save the tag ids
          std::vector<string> tag_ids;
          // if we also navigate for finding a tag
          if (norm_w_rad_variance > 0) {
            // Give 5 second to make radiation sensor stabilise
            ros::Duration(5).sleep();
            // if(radiation_level_client.waitForExistence(ros::Duration(5.0))){
            //   if (radiation_level_client.call(radiation_level_srv)) {
            //     cout << "   RadiationLevel client answered" << endl;
            //     current_rad_level = radiation_level_srv.response.rad_level;
            //     content = to_string(target.getX()) + "," +
            //               to_string(target.getY()) + "," + 
            //               to_string(current_rad_level) + "\n";
            //     utils.filePutContents(rad_readings_log, content, true);
            //   }
            // }
            
            // Obtain a reading calling the GP service
            gp_node::AddReading add_reading_srv;
            cout << target.getX() << " " << target.getY() << endl;
            add_reading_srv.request.reading_x_loc = target.getX();
            add_reading_srv.request.reading_y_loc = target.getY();
            if (add_readings_client.call(add_reading_srv)){
              cout << "   [ORI] Reading correctly sampled!" << endl;
              current_rad_level = add_reading_srv.response.reading_value;
              content = to_string(target.getX()) + "," +
                        to_string(target.getY()) + "," + 
                        to_string(current_rad_level) + "\n";
              utils.filePutContents(rad_readings_log, content, true);
            }else cout << "   [ORI] ERROR in getting a reading." << endl;
            
            printf("Updating the predictions ...\n");
            // Now, call the service for obtaining a prediction at each topological node
            gp_node::GetPredictions get_prediction_srv;
            if (get_predictions_client.call(get_prediction_srv)){
              prediction_tools.mean_values = get_prediction_srv.response.mean_predictions;
              prediction_tools.var_values = get_prediction_srv.response.variance_predictions;
              if (count == 0){
                initial_map_variance = accumulate(prediction_tools.var_values.begin(), prediction_tools.var_values.end(), 0.0);
                current_map_variance = initial_map_variance;
              }else {
                current_map_variance = accumulate(prediction_tools.var_values.begin(), prediction_tools.var_values.end(), 0.0);
              }
              current_map_variance = current_map_variance / initial_map_variance;
              cout << "   [ORI] Prediction correctly obtained by the GP node." << endl;
              cout << "   Initial variance: " << initial_map_variance << ", current variance: " << current_map_variance 
                   << "[" << 100 * (variance_ratio) << "]" << endl;
            }else cout << "   [ORI] ERROR in getting the predictions." << endl;         
          }
          

          // If the exploration just started
          if (count == 0) {
            // Calculate other three pose given the starting one
            string invertedPose = function.getEncodedKey(invertedInitial, 0);
            string eastPose = function.getEncodedKey(eastInitial, 0);
            string westPose = function.getEncodedKey(westInitial, 0);
            list<Pose> empty;
            std::pair<string, list<Pose>> pair1 =
                make_pair(invertedPose, empty);
            std::pair<string, list<Pose>> pair2 = make_pair(eastPose, empty);
            std::pair<string, list<Pose>> pair3 = make_pair(westPose, empty);
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
            for (auto cw_it = closest_waypoints.begin(); cw_it != closest_waypoints.end(); cw_it++){
              utils.pushInitialPositions(*cw_it,
                                         map, x, y, orientation, range, FOV, threshold, actualPose,
                                         &graph2, &topo_path_client, &mapping_time_belief, &function, &batteryTime,
                                         &radiation_mean_map, &mappingWaypoints, &prediction_tools, &distances_map);
            }
          }

          list<Pose> frontiers = topoMap;
          // If there are no new candidate positions from the current pose of
          // the robot
          if (frontiers.size() == 0) {
            std::cout << "[ERROR] Topological map empty." << endl;
            break;
          }
          //... otherwise, if there are further candidate new position from the
          // current pose of the robot
          else {
            // Remove the closest cells to every tag, to avoind investing the agent!
            // unordered_map<string, string>::iterator it = mappingWaypoints.begin(); // this contains (encoding, waypoint name)
            for (std::pair<string,string> element : mappingWaypoints){
              if (std::find(closest_waypoints.begin(), closest_waypoints.end(), element.second) != closest_waypoints.end()){
                Pose rm_pose = record.getPoseFromEncoding(element.first);
                frontiers.remove(rm_pose);
                std::cout << "\tRemoved node because the picker is on it: " << element.second << std::endl;
              }
            }


            // Remove the cells which are recently visited (part of tabuList)
            list<Pose>::iterator it_tabuList;
            for (it_tabuList = tabuList.begin(); it_tabuList != tabuList.end();
                 it_tabuList++) {
              utils.cleanPossibleDestination2(&frontiers, *it_tabuList);
            }
            // FIXME: this can still be useful by not every iteration
            utils.cleanDestinationFromTabulist(&frontiers, &posToEsclude);
            // cout <<"Analysing all possible destinations : " << frontiers.size() << endl;
            // mapping_time_belief = utils.getStatelessRFIDBelief(180.0, true, &pf_stateless_likelihoodClient_list);
            cout << "Obtain current robot waypoint name" << endl;
            string rob_closerWaypoint;
            utils.getModelClosestWaypoint(robotName, topological_map, rob_closerWaypoint, gt_tag_pose);
            cout << "Evaluating nodes..." << endl;
            start = ros::Time::now().toSec();
            record = *function.evaluateFrontiers(rob_closerWaypoint, 
                frontiers, map, threshold, topo_path_client, mapping_time_belief, batteryTime,
                radiation_mean_map, mappingWaypoints, prediction_tools, distances_map);
            cout << "   Evaluation: " << ros::Time::now().toSec() - start << endl;
            // FIXME: this shouldn't be necessary but I cannot remove it because
            // some cells in the tabulist are not removed with
            // cleanPossibleDestination2 Clean all the possible destination from
            // cells recently visited
            for (list<Pose>::iterator it = tabuList.begin();
                 it != tabuList.end(); it++) {
              record.removeFrontier(*it);
            }
            // If there are candidate positions
            if (record.size() > 0) {
              frontiers = record.getFrontiers();
              // Set the previous pose equal to the current one (represented by
              // target)
              previous = target;
              // Select the new robot destination from the list of candidates
              std::pair<Pose, double> result = function.selectNewPose(&record);
              target = result.first;
              // target = utils.selectFreePoseInLocalCostmap(
              //     target, &frontiers, &map, &function, threshold,
              //     &topo_path_client, &posToEsclude, &record,
              //     move_base_local_costmap_topic_name, &batteryTime, &radiation_map,
              //     &mappingWaypoints, &belief_topomaps);
              targetPos =
                  std::make_pair(int(target.getX()), int(target.getY()));
              // Select a new position until it is not explored recently
              int iter = 0;
              while (utils.containsPos(&posToEsclude, targetPos)) {
                iter++;
                // cout << iter << endl;
                if (record.size() > 0) {
                  record.removeFrontier(target);
                  result = function.selectNewPose(&record);
                  target = result.first;
                  // target = utils.selectFreePoseInLocalCostmap(
                  //     target, &frontiers, &map, &function, threshold,
                  //     &topo_path_client, &posToEsclude, &record,
                  //     move_base_local_costmap_topic_name, &batteryTime,
                  //     &radiation_map, &mappingWaypoints, &belief_topomaps);
                  targetPos =
                      std::make_pair(int(target.getX()), int(target.getY()));
                  posToEsclude.push_back(targetPos);
                } else {
                  cout << "Record is empty and cannot find new candidate pose. Navigation should be terminated." << endl;
                  break_loop = true;
                  break;
                }
              }

              if (break_loop == true)
                break;

              // If the selected destination does not appear among the cells
              // already visited
              if ((!utils.containsPos(&posToEsclude, targetPos))) {

                // i switch x and y to allow debugging graphically looking the image
                // cout << "New target : x = " << targetPos.first << ", y = " << targetPos.second 
                //     << ", orientation = " << target.getOrientation() * 180 / M_PI << endl;
                // Add it to the list of visited cells as first-view
                encodedKeyValue = 1;
                backTracking = false;

                // If the target has a different orientation from the current one,
                // let's rotate the robot first
                // cout << "Current Orientation: " << previous.getOrientation() << 
                //   ", target Orientation: " << target.getOrientation() << endl;
                // if (fmod(M_PI, std::abs(previous.getOrientation() - target.getOrientation())) > M_PI/6 ){
                //   cout << "Let's rotate the robot." << endl;
                //   utils.move(previous.getX(), 
                //             previous.getY(), 
                //             target.getOrientation(), 
                //             20,
                //             &tabuList,
                //             &posToEsclude);
                //   cout << "Rotation completed! Sending waypoint now." << endl;
                // }
                cout << "New destination found." << endl;
                success = utils.showMarkerandNavigate(
                    target, &marker_pub, &map, &topo_path_client, &tabuList,
                    &posToEsclude, TRANSL_SPEED, &batteryTime,
                    &travelledDistance, &mappingWaypoints, topological_map, tag_ids);
                if (success == true) {
                  // Recently visited cells shouldn't be visited soon again
                  if (tabuListCount > 0) {
                    tabuListCount--;
                  } else {
                    // cout << "----> RESET TABULIST!! <----" << endl;
                    // tabuList.clear();
                    // posToEsclude.clear();
                    // tabuListCount = MAX_TABULIST_COUNT;
                    
                    // NOTE: instead of emptying the list allowing to sample a cell 
                    // visited only two steps ago, only remove the oldest waypoint
                    tabuList.pop_front();
                    posToEsclude.clear();
                  }
                  // We empty the lists before pushing the new target inside, 
                  // so it can't be selected at the next NBS iteration
                  utils.updatePathMetrics(
                      &count, &target, &previous, actualPose, &frontiers, &graph2,
                      &map, &function, &tabuList, &posToEsclude, &history,
                      encodedKeyValue, &numConfiguration, &totalAngle,
                      &travelledDistance, &numOfTurning, scanAngle,
                      &topo_path_client, backTracking, robot_radius, &mappingWaypoints);
                }
                // Update the number of configurations of the robot along the task
                numConfiguration++;
                // Update counter of iterations
                count++;
                scan = true;
              }
              // ...otherwise, if the selected cell has already been visited
              else {
                // NOTE: Technically, it shouldn't enter here due to the
                // previous while loop
                //                                cout << "3" << endl;
                cout << "We shouldn't be here" << endl;
                exit(0);
              }
            }
            // ... otherwise, if there are no candidate positions
            else {
              cout << "There are no more waypoints significant to explore. End "
                      "the exploration task."
                   << endl;
              break;
            }

            // If the record is empty, we didn't find a new position so we must finish
            if (break_loop == true)
              break;
            // NOTE: not requested for testing purpose
            // usleep(microseconds);
            sensedCells = newSensedCells;
            frontiers.clear();
            // delete record;
          }
        }

        // after 1 loop is complete, publish some stats
        // IF YOU CHANGE WHAT'S PUBLISHED HERE CHANGE HEADER PUBLISHING ABOVE!!!
        coverage = 100 * float(newSensedCells) / float(totalFreeCells);
        batteryPercentage = 100 * batteryTime / MAX_BATTERY;
        stats_buffer.str(std::string()); // remove old data
        stats_buffer << (coverage) << ", " << (numConfiguration) << ", "
                     << (btMode);
        stats_msg.data = stats_buffer.str();

        stats_pub.publish(stats_msg);

      }
      // Perform exploration until a certain stopping criterion is achieved
      while (batteryPercentage > 1.0 and count < max_iterations and variance_ratio > gp_var_threshold);
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
      list<Pose> tmp_history = utils.cleanHistory(&history, &record);
      utils.calculateDistance(tmp_history, &topo_path_client, robot_radius);

      cout << "------------------ TABULIST -----------------" << endl;
      utils.calculateDistance(tabuList, &topo_path_client, robot_radius);

      cout << "------------------- ending experiment ---------------" << endl;
      std_msgs::Bool finished;
      finished.data = true;
      experiment_finished_pub.publish(finished);

      utils.printResult(newSensedCells, totalFreeCells, precision,
                        numConfiguration, travelledDistance, numOfTurning,
                        totalAngle, totalScanTime, resolution, norm_w_info_gain,
                        norm_w_travel_distance, norm_w_rad_mean, 
                        norm_w_battery_status, norm_w_rad_variance, out_log);
      // Find the tag
      std::vector<std::pair<int, std::pair<int, int>>> tag_positions =
          utils.findTagFromBeliefMap(&radiation_mean_map);
      cout << "Tags positions (for all the found ones):" << endl;
      for (int i = 0; i < tag_positions.size(); i++) {
        std::pair<int, std::pair<int, int>> tag = tag_positions.at(i);
        cout << "   Tag[" << to_string(i) << "] = (" << tag.second.first << ","
             << tag.second.second << ")" << endl;
      }
      cout
          << "-----------------------------------------------------------------"
          << endl;
      auto endMCDM = ros::Time::now().toSec();

      double totalTimeMCDM = endMCDM - startMCDM;
      cout << "Total time for MCDM algorithm : " << totalTimeMCDM << "s, "
           << totalTimeMCDM / 60 << " m " << endl;
      cout << "Spinning at the end" << endl;



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
    // cout << "CALLBACK FIRST!" << endl;
    costmap_grid = *msg;
    costresolution = msg->info.resolution;
    costwidth = msg->info.width;
    costheight = msg->info.height;
    costorigin = msg->info.origin;
    for (int i = 0; i < msg.get()->data.size(); ++i) {
      occdata.push_back(msg->data.at(i));
    }
    // std::cout << "size of occdata " << occdata.size()
    //           << " size of message data " << msg->data.size() << std::endl;
    // std::cout << "height " << msg->info.height << " width " << msg->info.width
    //           << " resolution " << msg->info.resolution << std::endl;
    costmapReceived = 1;
  }
}

void belief_topomap_callback(const bayesian_topological_localisation::DistributionStampedConstPtr &msg){
  belief_counter = belief_counter + 1;
  cout << "ATTENZIONE" << endl;
  // string tag = msg->header.frame_id;
  // cout << "[belief_topomap_callback@pure_navigation.cpp]tag: " << tag << endl;
  // int len = sizeof(tag)/sizeof(tag[0]);
  // // std::string::size_type pos = tag.find(tag.end());
  // int index = std::stoi(tag.substr(3, len));  // let's drop "tag_" substring
  // cout << "[belief_topomap_callback@pure_navigation.cpp]index: " << index << endl;
  // If there are not maps, push it back to avoid overflow, otherwise access the right index
  // if (index != 0){
  //   if (belief_topomaps.size() < index){
  //     belief_topomaps.push_back(*msg);
  //   }else belief_topomaps.at(index-1) = *msg;
  // }
}

void topological_map_callback(const strands_navigation_msgs::TopologicalMapConstPtr &msg){
  if (topoMapReceived == 0){
    topological_map = *msg;
    topoMapReceived = 1;
    // cout << "[TOPOLOGICALMAP] Number of elements: " << topological_map.nodes.size() << endl;
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
  private_node_handle.param("radiation_map_srv_name", radiation_map_srv_name, std::string("/get_radiation_map"));
  // private_node_handle.param("fake_radiation_map_srv_name", fake_radiation_map_srv_name, std::string("/rfid_grid_map_node/get_fake_rfid_belief"));
  private_node_handle.param("/move_base/global_costmap/robot_radius", robot_radius, 0.25);
  private_node_handle.param("make_plan_srv_name", make_plan_srv_name, std::string("/move_base/make_plan"));
  private_node_handle.param("make_topo_plan_srv_name", make_topo_plan_srv_name, std::string("/get_simple_policy/get_route_to"));
  private_node_handle.param("get_topo_distances_srv_name", get_topo_distances_srv_name, std::string("/get_simple_policy/get_route_between"));
  private_node_handle.param("move_base_goal_topic_name", move_base_goal_topic_name, std::string("move_base_simple/goal"));
  private_node_handle.param("move_base_srv_name", move_base_srv_name, std::string("move_base"));
  private_node_handle.param("nav_grid_debug_topic_name", nav_grid_debug_topic_name, std::string("nav_grid_debug"));
  private_node_handle.param("planning_grid_debug_topic_name", planning_grid_debug_topic_name, std::string("planning_grid_debug"));
  private_node_handle.param("move_base_costmap_topic_name", move_base_costmap_topic_name, std::string("move_base/global_costmap/costmap"));
  private_node_handle.param("move_base_costmap_updates_topic_name", move_base_costmap_updates_topic_name, std::string("move_base/global_costmap/costmap_updates"));
  private_node_handle.param("move_base_local_costmap_topic_name", move_base_local_costmap_topic_name, std::string("/move_base/local_costmap/costmap"));
  private_node_handle.param("marker_pub_topic_name", marker_pub_topic_name, std::string("goal_pt"));
  private_node_handle.param("rosbag_srv_name", rosbag_srv_name, std::string("/record/cmd"));
  private_node_handle.param("stats_topic_name", stats_topic_name, std::string("/mcdm_stats"));
  private_node_handle.param("topological_map_topic_name", topological_map_topic_name, std::string("/topological_map"));
  private_node_handle.param("localization_srv_name", localization_srv_name, std::string("/bayesian_topological_localisation/localise_agent"));
  private_node_handle.param("pf_topic_name", pf_topic_name, std::string("/prob_dist_obs"));
  private_node_handle.param("gazebo_model_state_srv_name", gazebo_model_state_srv_name, std::string("/gazebo/get_model_state"));
  private_node_handle.param("experiment_finished_topic_name", experiment_finished_topic_name, std::string("/experiment_finished"));
  private_node_handle.param("radiation_level_srv_name", radiation_level_srv_name, std::string("/get_radiation_level"));
  private_node_handle.param("set_query_pts_srv_name", set_query_pts_srv_name, std::string("/gp_node/set_query_points"));
  private_node_handle.param("add_reading_srv_name", add_reading_srv_name, std::string("/gp_node/add_reading"));
  private_node_handle.param("get_predictions_srv_name", get_predictions_srv_name, std::string("/gp_node/get_predictions"));
}

void printROSParams(){
  printf("/////////////////////////////////////////////////////////////////////////\n");
  printf("[pure_navigation@printROSParams] Using the following ros params:\n");
  printf("   - robot_radius [%3.3f]\n",  robot_radius);
  printf("   - static_map_srv_name [%s]\n", static_map_srv_name.c_str());
  printf("   - radiation_map_srv_name [%s]\n", radiation_map_srv_name.c_str());
  printf("   - radiation_level_srv_name [%s]\n", radiation_level_srv_name.c_str());
  printf("   - set_query_pts_srv_name [%s]\n", set_query_pts_srv_name.c_str());
  printf("   - set_query_pts_srv_name [%s]\n", set_query_pts_srv_name.c_str());
  printf("   - get_predictions_srv_name [%s]\n", get_predictions_srv_name.c_str());
  // printf("   - fake_radiation_map_srv_name [%s]\n", fake_radiation_map_srv_name.c_str());
  printf("   - make_plan_srv_name [%s]\n", make_plan_srv_name.c_str());
  printf("   - make_topo_plan_srv_name [%s]\n", make_topo_plan_srv_name.c_str());
  printf("   - move_base_goal_topic_name [%s]\n", move_base_goal_topic_name.c_str());
  printf("   - move_base_srv_name [%s]\n", move_base_srv_name.c_str());
  printf("   - nav_grid_debug_topic_name [%s]\n", nav_grid_debug_topic_name.c_str());
  printf("   - planning_grid_debug_topic_name [%s]\n", planning_grid_debug_topic_name.c_str());
  printf("   - move_base_costmap_topic_name [%s]\n", move_base_costmap_topic_name.c_str());
  printf("   - move_base_costmap_updates_topic_name [%s]\n", move_base_costmap_updates_topic_name.c_str());
  printf("   - marker_pub_topic_name [%s]\n", marker_pub_topic_name.c_str());
  printf("   - stats_topic_name [%s]\n", stats_topic_name.c_str());
  printf("   - topological_map_topic_name [%s]\n", topological_map_topic_name.c_str());
  printf("   - pf_topic_name [%s]\n", pf_topic_name.c_str());
  printf("   - experiment_finished_topic_name [%s]\n", experiment_finished_topic_name.c_str());

  printf("/////////////////////////////////////////////////////////////////////////\n");

}

ros::NodeHandle createROSComms(){

  ros::NodeHandle nh;
  ros::Rate r(20);
  bool disConnected = true;

  // create service clients
  map_service_client_ = nh.serviceClient<nav_msgs::GetMap>(static_map_srv_name);
  path_client = nh.serviceClient<nav_msgs::GetPlan>(make_plan_srv_name, true);
  topo_path_client = nh.serviceClient<strands_navigation_msgs::GetRouteTo>(make_topo_plan_srv_name, true);
  topo_distances_client = nh.serviceClient<strands_navigation_msgs::GetRouteBetween>(get_topo_distances_srv_name, true);
  // radiation_map_client = nh.serviceClient<nav_msgs::GetMap>(radiation_map_srv_name);
  radiation_map_client = nh.serviceClient<ncnr_logging::GetRadiationPredictions>(radiation_map_srv_name);
  radiation_level_client = nh.serviceClient<ncnr_logging::GetRadiationLevel>(radiation_level_srv_name);
  set_query_pts_client = nh.serviceClient<gp_node::SetQueryPoints>(set_query_pts_srv_name);
  add_readings_client = nh.serviceClient<gp_node::AddReading>(add_reading_srv_name);
  get_predictions_client = nh.serviceClient<gp_node::GetPredictions>(get_predictions_srv_name);
  // fake_radiation_map_client = nh.serviceClient<rfid_grid_map::GetFakeBeliefMaps>(fake_radiation_map_srv_name);
  localization_client = nh.serviceClient<bayesian_topological_localisation::LocaliseAgent>(localization_srv_name);
  gazebo_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>(gazebo_model_state_srv_name);
  //rosbag_client = nh.serviceClient<record_ros::String_cmd>(rosbag_srv_name);

  // create publishers
  moveBasePub =   nh.advertise<geometry_msgs::PoseStamped>(move_base_goal_topic_name, 1000);
  gridPub = nh.advertise<grid_map_msgs::GridMap>(nav_grid_debug_topic_name, 1, true);
  planningPub = nh.advertise<grid_map_msgs::GridMap>(planning_grid_debug_topic_name, 1, true);
  marker_pub =  nh.advertise<visualization_msgs::Marker>(marker_pub_topic_name, 10);
  stats_pub =  nh.advertise<std_msgs::String>(stats_topic_name, 1, true);
  experiment_finished_pub = nh.advertise<std_msgs::Bool>(experiment_finished_topic_name, 1, true);
  rad_mean_grid_pub = nh.advertise<grid_map_msgs::GridMap>("radiation_mean_grid_map", 1000);
  rad_var_grid_pub = nh.advertise<grid_map_msgs::GridMap>("radiation_var_grid_map", 1000);
  // Create srv client for fake likelihood readings
  // prediction_tools.radarmodel_fake_reading_srv_list.push_back(fake_radiation_map_client);

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
  tag_coverage_sub = nh.subscribe<std_msgs::Float32>("tag_coverage", 10, tag_coverage_callback);
  topo_map_sub = nh.subscribe<strands_navigation_msgs::TopologicalMap>("/topological_map", 10, topological_map_callback);
  // radiation_map_sub = nh.subscribe<grid_map_msgs::GridMap>("rfid_radiation_maps", 10, radiation_map_callback);
  // pf_sub = nh.subscribe<bayesian_topological_localisation::DistributionStamped>("/tag_1/current_prob_dist", 1, belief_topomap_callback, ros::TransportHints().tcpNoDelay());  
  // gps_sub = nh.subscribe<visualization_msgs::Marker>("/tag_1/gps_pose", 10, gps_callback);
  return nh;
}


void tag_coverage_callback(const std_msgs::Float32 msg){
  tag_coverage_percentage = msg.data;
  // cout << "   [CALLBACK] : " << tag_coverage_callback << endl;
}

// void radiation_map_callback(const grid_map_msgs::GridMap msg){
//   converter.fromMessage(msg, radiation_mean_map);
// }

void gps_callback(const visualization_msgs::MarkerConstPtr &msg){
  gps_tag_pose = msg->pose;
}


// void sensing(){
//     ros::NodeHandle nh("~");
//     ros::Subscriber rfid_sub;
//     rfid_sub = nh.subscribe<std_msgs::Float32>("tag_coverage", 10, tag_coverage_callback);
//     ros::AsyncSpinner spinner(0);
//     spinner.start();
//     auto start = chrono::high_resolution_clock::now();
//     // gasDetection();
//     while(ros::ok()){
//         ROS_INFO("RFID sensing ...");
//     }

// }