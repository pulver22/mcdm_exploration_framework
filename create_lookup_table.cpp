#define _USE_MATH_DEFINES
#include "utils.h"
#include <algorithm>
#include <iostream>
#include <iterator>
#include "math.h"
#include <ctime>
#include <time.h>
#include <unistd.h>
#include <ctime>
#include <iomanip>
#include "movebasegoal.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <ros/console.h>
#include "strands_navigation_msgs/TopologicalMap.h"
#include "strands_navigation_msgs/GetRouteBetween.h"
#include <experimental/filesystem> // or #include <filesystem> for C++17 and up
#include "nav_msgs/OccupancyGrid.h"
#include "grid_map_msgs/GridMap.h"

using namespace std;

namespace fs = std::experimental::filesystem;

// ROS varies
void grid_callback(const nav_msgs::OccupancyGridConstPtr &msg);
void topological_map_callback(const strands_navigation_msgs::TopologicalMapConstPtr &msg);
void loadROSParams();
ros::NodeHandle createROSComms();


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>MoveBaseClient;
strands_navigation_msgs::TopologicalMap topological_map;

vector<bayesian_topological_localisation::DistributionStamped> belief_topomaps;

//  ROS PARAMETERS ....................................
std::string make_plan_srv_name;
std::string get_topo_distances_srv_name;
std::string move_base_goal_topic_name;
std::string move_base_srv_name;
std::string topological_map_topic_name;
nav_msgs::OccupancyGrid costmap_grid;

ros::Subscriber topo_map_sub;


// Ros services/subscribers/publishers
ros::ServiceClient map_service_client_;
ros::ServiceClient path_client;
ros::ServiceClient topo_distances_client;
nav_msgs::GetMap srv_map;

int topoMapReceived = 0;



int main(int argc, char **argv) {

  ros::init(argc, argv, "lookup_table_node");
  loadROSParams();
  // create ROS connections/services
  ros::NodeHandle nh = createROSComms();
  ros::Rate r(20);
  
  double initFov = atof(argv[1]);
  initFov = initFov * M_PI / 180;
  int initRange = atoi(argv[2]);
  std::string path_distances_map = (argv[3]);
  std::string log_dest_folder = (argv[4]);
  string map_path = log_dest_folder + path_distances_map ;
  Utilities utils;

  while (ros::ok()) {

    
    if (topoMapReceived == 0) {
      ROS_INFO_STREAM_THROTTLE(60, "waiting for topologicalmap" << std::endl);
    }
    if (topoMapReceived == 1) {
      list<Pose> topoMap;
      unordered_map<string, string> mappingWaypoints;
      utils.convertStrandTopoMapToListPose(
          &topological_map, &topoMap, initRange, initFov, &mappingWaypoints);
      ROS_DEBUG("TopologicalMap created");

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
        cout << "Saving in: " << map_path << endl;
        utils.saveMap(&distances_map, map_path);
        cout << "Shutting down now..." << endl;
        ros::shutdown();
      }    
    }
    ros::spinOnce();
    r.sleep();
  } // end while ros::ok
} // end main

// void grid_callback(const nav_msgs::OccupancyGridConstPtr &msg) {
//   // printf("RECEIVED A MAP!");
//   if (costmapReceived == 0) {
//     // cout << "CALLBACK FIRST!" << endl;
//     costmap_grid = *msg;
//     costresolution = msg->info.resolution;
//     costwidth = msg->info.width;
//     costheight = msg->info.height;
//     costorigin = msg->info.origin;
//     for (int i = 0; i < msg.get()->data.size(); ++i) {
//       occdata.push_back(msg->data.at(i));
//     }
//     // std::cout << "size of occdata " << occdata.size()
//     //           << " size of message data " << msg->data.size() << std::endl;
//     // std::cout << "height " << msg->info.height << " width " << msg->info.width
//     //           << " resolution " << msg->info.resolution << std::endl;
//     costmapReceived = 1;
//   }
// }


void topological_map_callback(const strands_navigation_msgs::TopologicalMapConstPtr &msg){
  if (topoMapReceived == 0){
    topological_map = *msg;
    topoMapReceived = 1;
  }
}

void loadROSParams(){

  ros::NodeHandle private_node_handle("~");
  // LOAD ROS PARAMETERS ....................................
//   private_node_handle.param("static_map_srv_name", static_map_srv_name, std::string("static_map"));
  private_node_handle.param("get_topo_distances_srv_name", get_topo_distances_srv_name, std::string("/get_simple_policy/get_route_between"));
  private_node_handle.param("topological_map_topic_name", topological_map_topic_name, std::string("/topological_map"));
}

ros::NodeHandle createROSComms(){

  ros::NodeHandle nh;
  ros::Rate r(20);
  bool disConnected = true;

  // create service clients
//   map_service_client_ = nh.serviceClient<nav_msgs::GetMap>(static_map_srv_name);
  topo_distances_client = nh.serviceClient<strands_navigation_msgs::GetRouteBetween>(get_topo_distances_srv_name, true);
  
//   while (disConnected) {
//     cout << "[pure_navigation@createROSComms] Waiting for static_map service to respond..." << endl;
//     if (map_service_client_.call(srv_map)) {
//       costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>(
//            move_base_costmap_topic_name, 100, grid_callback);
//       disConnected = false;
//     } else {
//       r.sleep();
//     }
//   }
  topo_map_sub = nh.subscribe<strands_navigation_msgs::TopologicalMap>("/topological_map", 10, topological_map_callback);
  return nh;
}


