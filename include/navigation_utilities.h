//
// Created by pulver on 20/07/19.
//

#ifndef NAVIGATION_UTILITIES_H
#define NAVIGATION_UTILITIES_H

#include "map.h"
#include "pose.h"
#include "mcdmfunction.h"
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
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/console.h>
#include "record_ros/record.h"
#include "record_ros/String_cmd.h"

using namespace std;
using namespace dummy;


class NavigationUtilities {

  public:
    NavigationUtilities();
    ~NavigationUtilities();
    bool contains(std::list<Pose> &list, Pose &p);

    bool containsPos(std::list<std::pair<float, float>>* positionEscluded,
                     std::pair<float, float> p);

    void cleanPossibleDestination2(std::list<Pose> *possibleDestinations, Pose &p);

    void cleanDestinationFromTabulist(std::list<Pose> *possibleDestinations, std::list<std::pair<float, float> > *posToEsclude);

    void pushInitialPositions(dummy::Map map, float x, float y, float orientation,
                              int range, int FOV, double threshold,
                              string actualPose,
                              vector<pair<string, list<Pose> >> *graph2,
                              ros::ServiceClient *path_client,
                              MCDMFunction *function);

    double calculateScanTime(double scanAngle);

    void calculateDistance(list<Pose> history, ros::ServiceClient *path_client, double robot_radius);

    Pose createFromInitialPose(Pose pose, float variation, int range, int FOV);

    void updatePathMetrics(int *count, Pose *target, Pose *previous, string actualPose,
                            list<Pose> *nearCandidates, vector<pair<string, list<Pose> >> *graph2,
                            dummy::Map *map, MCDMFunction *function, list<Pose> *tabuList,
                            list<pair<float, float> > *posToEsclude, vector<string> *history,
                            int encodedKeyValue, long *numConfiguration,
                            double *totalAngle, double *travelledDistance, int *numOfTurning,
                            double scanAngle, ros::ServiceClient *path_client, bool backTracking, double robot_radius);

    double getAngleBetPoses(geometry_msgs::PoseStamped ps1, geometry_msgs::PoseStamped ps2);

    list<Pose> cleanHistory(vector<string> *history, EvaluationRecords *record_history);

    void printResult(long newSensedCells, long totalFreeCells, double precision,
                     long numConfiguration, double travelledDistance,
                     int numOfTurning, double totalAngle, double totalScanTime, double resoion,
                     float w_info_gain, float w_travel_distance, float w_sensing_time, std::string fileURI);

    bool showMarkerandNavigate(Pose target, ros::Publisher *marker_pub,
                               nav_msgs::GetPlan *path,
                               ros::ServiceClient *path_client,
                               list<Pose> *tabuList,
                               std::list<std::pair<float, float> > *posToEsclude,
                               double min_robot_speed, double robot_radius);

    bool freeInLocalCostmap(Pose target, std::string move_base_local_costmap_topic_name);

    // ROS varies
    bool move(float x, float y, float orientation, float time_travel,
              list<Pose> *tabuList,
              std::list<std::pair<float, float> > *posToEsclude);


    Pose getCurrentPose(float resolution, float costresolution, dummy::Map *map,
                        double initFov, int initRange);

    double getPathLen(std::vector<geometry_msgs::PoseStamped> poses, double robot_radius);

    Pose selectFreePoseInLocalCostmap(Pose target, list<Pose> *nearCandidates, dummy::Map *map, MCDMFunction *function,
                                      double threshold, ros::ServiceClient *path_client,
                                      std::list<std::pair<float, float> > *posToEsclude, EvaluationRecords *record,
                                      std::string move_base_local_costmap_topic_name);

    void saveCoverage(const std::string& name, const std::string& content, bool append);
};


#endif //NAVIGATION_UTILITIES_H
