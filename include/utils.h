//
// Created by pulver on 20/07/19.
//

#ifndef UTILITIES_H
#define UTILITIES_H

#include "map.h"
#include "mcdmfunction.h"
#include "pose.h"
#include <algorithm>
#include <iostream>
#include <iterator>

#define _USE_MATH_DEFINES

#include "math.h"
#include <ctime>
#include <time.h>
#include <unistd.h>
// #include "RFIDGridmap.h"
#include "bayesian_topological_localisation/DistributionStamped.h"
#include "movebasegoal.h"
#include "strands_navigation_msgs/TopologicalMap.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "topological_navigation/GotoNodeAction.h"
#include "topological_navigation/GotoNodeActionGoal.h"
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetModelStateRequest.h>
#include "strands_navigation_msgs/TopologicalMap.h"
#include "visualization_msgs/Marker.h"
#include "Criteria/criterion.h"
// mfc: we will record using stats_pub
//#include "record_ros/record.h"
//#include "record_ros/String_cmd.h"

using namespace std;
using namespace dummy;
using namespace grid_map;

class Utilities {

private:
    actionlib::SimpleActionClient<topological_navigation::GotoNodeAction> *topoAC;
    ros::ServiceClient *gazebo_model_state_client;
    visualization_msgs::Marker goal_marker_;
    Criterion criterion_utils_;
    EvaluationRecords record_;
    string closest_waypoint_name_;
    geometry_msgs::Pose closest_waypoint_pose_;

public:
    Utilities();
    ~Utilities();

    /**
   * Check if a Pose object is already present within a list
   *
   * @param list: the list of Pose objects
   * @param p: the Pose object to look for
   */
    bool contains(std::list<Pose> &list, Pose &p);

    bool containsPos(std::list<std::pair<float, float>> *positionEscluded,
                     std::pair<float, float> p);
    /**
   * Remove a Pose object from a list of Pose objects
   *
   *  @param possibleDestinations: list of Pose objects
   * @param p: the object to removeS
   */
    void cleanPossibleDestination2(std::list<Pose> *possibleDestinations,
                                   Pose &p);

    void cleanDestinationFromTabulist(
        std::list<Pose> *possibleDestinations,
        std::list<std::pair<float, float>> *posToEsclude);

    /**
   * Calculate the frontiers from the starting position and add it to the graph
   * structure
   *
   * @param map: a copy of the map
   * @param x: the x-coordinate of the robot expressed in cells
   * @param y: the y-coordinate of the robot expressed in cells
   * @param orientation: the orientation of the robot expressed in degrees
   * @param range: the range of the robot's sensor expressed in cells
   * @param threshold: the threshold value to discard unuseful frontiers
   * @param actualPose: encoding of the current robot pose
   * @param graph2: the structure where to save current pose and its candidate
   * positions
   * @param function: the MCDM function object
   */
    void pushInitialPositions(string currentRobotWayPoint, 
        dummy::Map map, float x, float y, float orientation, int range, int FOV,
        double threshold, string actualPose,
        vector<pair<string, list<Pose>>> *graph2, ros::ServiceClient *path_client,
        vector<unordered_map<float,
                             std::pair<string, bayesian_topological_localisation::
                                                   DistributionStamped>>>
            *mapping_time_belief,
        MCDMFunction *function, double *batteryTime, GridMap *belief_map,
        unordered_map<string, string> *mappingWaypoints, prediction_tools *tools,
        std::unordered_map<string, double> *distances_map);
    /**
   * Calculate the time required for performing a scan with the TDLAS sensor
   *
   * @param scanAngle: the angle to scan
   */
    double calculateScanTime(double scanAngle);

    double calculateDistanceMeters(list<Pose> history, ros::ServiceClient *path_client,
                           double robot_radius);

    /**
   * Create a new Pose object starting from another one
   *
   * @param x: the x-coordinate of the robot expressed in cell
   * @param y: the y-coordinate of the robot expressed in cell
   * @param orientation: the orientation of the robot expressed in degrees
   * @param range: the range of the robot's sensor expressed in cells
   * @param variation: an additional angle to sum to the orientation
   * @param FOV: the scanning angle of the sensor
   */
    Pose createFromInitialPose(Pose pose, float variation, int range, int FOV);

    void updatePathMetrics(
        int *count, Pose *target, Pose *previous, string actualPose,
        list<Pose> *nearCandidates, vector<pair<string, list<Pose>>> *graph2,
        dummy::Map *map, MCDMFunction *function, list<Pose> *tabuList,
        list<pair<float, float>> *posToEsclude, vector<string> *history,
        int encodedKeyValue, long *numConfiguration, double *totalAngle,
        double *travelledDistance, int *numOfTurning, double scanAngle,
        ros::ServiceClient *path_client, bool backTracking, double robot_radius,
        unordered_map<string, string> *mappingWaypoints);

    double getAngleBetPoses(geometry_msgs::PoseStamped ps1,
                            geometry_msgs::PoseStamped ps2);

    list<Pose> cleanHistory(vector<string> *history,
                            EvaluationRecords *record_history);

    void printResult(long newSensedCells, long totalFreeCells, double precision,
                     long numConfiguration, double travelledDistanceEdges, double travelledDistanceMeters,
                     int numOfTurning, double totalAngle, double totalScanTime,
                     double resoion, float w_info_gain, float w_travel_distance,
                     float w_rad_mean, float w_battery_status,
                     float w_rad_variance, float final_var_ratio, std::string fileURI);

    bool showMarkerandNavigate(Pose target, ros::Publisher *marker_pub,
                               dummy::Map *map,
                               ros::ServiceClient *path_client,
                               list<Pose> *tabuList,
                               std::list<std::pair<float, float>> *posToEsclude,
                               double min_robot_speed,
                               double *batteryTime, double *travelledDistance,
                               unordered_map<string, string> *mappingWaypoints,
                               strands_navigation_msgs::TopologicalMap topological_map,
                               std::vector<string> tag_ids);

    double computeMetricDistance(Pose target, dummy::Map *map, ros::ServiceClient *path_client);

    bool freeInLocalCostmap(Pose target,
                            std::string move_base_local_costmap_topic_name);

    // ROS varies
    bool move(float x, float y, float orientation, float time_travel,
              list<Pose> *tabuList,
              std::list<std::pair<float, float>> *posToEsclude);
    
    void setGazeboModelStateClient(ros::ServiceClient &gazebo_model_state_client);

    bool getGazeboModelPose(string model_name, string relative_entity_name, geometry_msgs::Pose *model_pose);

    bool getModelClosestWaypoint(
        string model_name, strands_navigation_msgs::TopologicalMap topological_map,
        string &closest_waypoint_name, geometry_msgs::Pose &closest_waypoint_pose);

    bool updateDestinationWaypoint(std::vector<string> tag_ids, 
            strands_navigation_msgs::TopologicalMap topological_map, string &waypointName,
            Pose &target, ros::Publisher &marker_pub,
            topological_navigation::GotoNodeActionGoal &topoGoal,
            bool &success, std::list<std::pair<float, float>> *posToEsclude);
    void checkOnNode(strands_navigation_msgs::TopologicalMap topological_map);

    bool moveTopological(Pose target, float time_travel, list<Pose> *tabuList,
                         std::list<std::pair<float, float>> *posToEsclude,
                         unordered_map<string, string> *mappingWaypoints,
                         strands_navigation_msgs::TopologicalMap topological_map,
                         std::vector<string> tag_ids, ros::Publisher *marker_pub);

    Pose getCurrentPose(float resolution, float costresolution, dummy::Map *map,
                        double initFov, int initRange);

    double getPathLen(std::vector<geometry_msgs::PoseStamped> poses,
                      double robot_radius);

    Pose selectFreePoseInLocalCostmap(string currentRobotWayPoint, 
        Pose target, list<Pose> *nearCandidates, dummy::Map *map,
        MCDMFunction *function, double threshold, ros::ServiceClient *path_client,
        vector<unordered_map<float,
                             std::pair<string, bayesian_topological_localisation::
                                                   DistributionStamped>>>
            *mapping_time_belief,
        std::list<std::pair<float, float>> *posToEsclude,
        EvaluationRecords *record, std::string move_base_local_costmap_topic_name,
        double *batteryTime, GridMap *belief_map,
        unordered_map<string, string> *mappingWaypoints, prediction_tools *tools, std::unordered_map<string, double> *distances_map);

    void filePutContents(const std::string &name, const std::string &content,
                         bool append);

    /**
   * Get the "guessed" position of the tags in the belief map
   *
   * @param belief_map: the probabilistic map built by sensor measurements
   * @return a vector of <likelikehood, tag_position(X,Y)> of all the tags in
   * the environment
   */
    std::vector<std::pair<int, std::pair<int, int>>>
    findTagFromBeliefMap(GridMap *belief_map);
    void convertStrandTopoMapToListPose(
        strands_navigation_msgs::TopologicalMap *topoMap, list<Pose> *frontiers,
        int range, double FoV, unordered_map<string, string> *mappingWaypoints);

    bayesian_topological_localisation::DistributionStamped
    convertGridBeliefMapToTopoMap(GridMap belief_map, list<Pose> topoMap,
                                  unordered_map<string, string> mappingWaypoints,
                                  string tag_id, double radius);

    // bool recordContainsCandidates(EvaluationRecords* record,
    //                           int* count, Pose* target, Pose* previous, string*
    //                           actualPose, list<Pose>* nearCandidates,
    //                           vector<pair<string,list<Pose>>>* graph2,
    //                           dummy::Map* map, MCDMFunction* function,
    //                           list<Pose>* tabuList, vector<string>* history,
    //                           int* encodedKeyValue, Astar* astar , long*
    //                           numConfiguration, double* totalAngle, double *
    //                           travelledDistance, int* numOfTurning , double*
    //                           scanAngle, bool* btMode, double* threshold,
    //                           double *batteryTime, bool *explorationCompleted);

    string getCloserWaypoint(geometry_msgs::Pose &pose,
                             strands_navigation_msgs::TopologicalMap topoMap);

    tuple<string, geometry_msgs::Pose> getCloserConnectedWaypoint(string node_name, Pose pose,
                                                                  strands_navigation_msgs::TopologicalMap *topoMap,
                                                                  std::list<std::pair<float, float>> *posToEsclude);

    geometry_msgs::Pose
    getWaypointPoseFromName(string name,
                            strands_navigation_msgs::TopologicalMap *topoMap);
    /*
   * Calculate a prediction for a maximum horizon (e.g, 50seconds) and
   * obtain a list of belief, one for each second. Pass this list to
   * RadMeanCriterion and seek the corresponding one for the candidate location in
   * exam.
   */
    vector<unordered_map<
        float, std::pair<string,
                         bayesian_topological_localisation::DistributionStamped>>>
    getStatelessRFIDBelief(
        double secs_from_now, bool return_history,
        vector<ros::ServiceClient> *pf_stateless_likelihoodClient_list);

    void saveMap(const std::unordered_map<string, double> *map, string path);
    bool loadMap(std::unordered_map<string, double> *map, string path);

    pair<bool, bayesian_topological_localisation::DistributionStamped> getRadiationDistribution(nav_msgs::OccupancyGrid grid, GridMapRosConverter converter, 
                                ros::Publisher *pub, list<Pose> topoMap, unordered_map<string, string> mappingWaypoints);
                            
    std::unordered_map<string, double> createDistanceTable(string map_path, ros::ServiceClient topo_distances_client, strands_navigation_msgs::TopologicalMap topological_map);
};

#endif // UTILITIES_H
