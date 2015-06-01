#include <iostream>
#include <iterator>
#include "map.h"
#include "ray.h"
#include "newray.h"
#include "mcdmfunction.h"
#include "graphpose.h"
#include "Criteria/traveldistancecriterion.h"
# define PI           3.14159265358979323846  /* pi */
#include <unistd.h>
#include <ros/ros.h>
#include "movebasegoal.h"
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>


using namespace std;
using namespace dummy;

int main(int argc, char **argv) {
    ros::init(argc, argv, "mcdm_exploration_framework_node");
    ros::NodeHandle nh;
    ros::ServiceClient map_service_client_ = nh.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv_map;
    ros::Publisher grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("mcdm_grid", 1000);
    
    // Input : ./mcdm_online_exploration_ros ./../Maps/map_RiccardoFreiburg_1m2.pgm 100 75 5 0 15 180 0.95 0.12
    // resolution x y orientation range centralAngle precision threshold
    
    while(ros::ok()){
    
    if (map_service_client_.call(srv_map)){
	ROS_INFO("Map service called successfully");
	const nav_msgs::OccupancyGrid& map (srv_map.response.map);
	ros::Publisher marker_pub = nh.advertise<geometry_msgs::PointStamped>("goal_pt", 10);
	float resolution = map.info.resolution;
	int width = map.info.width;
	int height = map.info.height;
	geometry_msgs::Pose origin = map.info.origin;
	vector<int> data;
	for(int i=0; i < map.data.size(); i++){
	    data.push_back(map.data.at(i));
	}
	Map newMap = Map(resolution,width,height,data,origin);
	
	
	
	nav_msgs::OccupancyGrid grid_msg;
	grid_msg.info.map_load_time = ros::Time::now();
	grid_msg.info.origin.position.x = 0;
	grid_msg.info.origin.position.y = 0;
	grid_msg.info.origin.position.z = 0;
	grid_msg.info.origin.orientation.w = 0;
	grid_msg.info.origin.orientation.x = 0;
	grid_msg.info.origin.orientation.y = 0;
	grid_msg.info.origin.orientation.z = 0;
	grid_msg.info.resolution = 0.05*(100/resolution);
	grid_msg.info.height = newMap.getNumGridRows();
	grid_msg.info.width = newMap.getNumGridCols();
	
	for(int n = 0; n < newMap.getNumGridCols()*newMap.getNumGridRows(); ++n){
	    grid_msg.data.push_back((int8_t)100*newMap.getGridValue(n));
	}
	
	grid_pub.publish< nav_msgs::OccupancyGrid>(grid_msg);
	
	
	long  initX = atoi(argv[2]);	
	long initY = atoi(argv[1]);
	int initOrientation = atoi(argv[3]);
	double initFov = atoi(argv[4] );
	initFov = initFov * PI /180;
	int initRange = atoi(argv[5]);
	double precision = atof(argv[6]);
	double threshold = atof(argv[7]);
	//x,y,orientation,range,angle -
	Pose initialPose = Pose(initX,initY,initOrientation,initRange,initFov);
	Pose target = initialPose;
	Pose previous = initialPose;
	long numConfiguration =0;
	//testing
	vector<pair<string,list<Pose>>> graph2;
	NewRay ray;
	MCDMFunction function;
	long sensedCells = 0;
	long newSensedCells =0;
	long totalFreeCells = newMap.getTotalFreeCells() ;
	int count = 0;
	int countBT;
	double travelledDistance = 0;
	unordered_map<string,int> visitedCell;
	vector<string>history;
	history.push_back(function.getEncodedKey(target,1));
	//amount of time the robot should do nothing for scanning the environment ( final value expressed in second)
	unsigned int microseconds = 5 * 1000 * 1000 ;
	//cout << "total free cells in the main: " << totalFreeCells << endl;
	list<Pose> unexploredFrontiers;
	while(sensedCells < precision * totalFreeCells ){
	long x = target.getX();
	long y = target.getY();
	int orientation = target.getOrientation();
	int range = target.getRange();
	double FOV = target.getFOV();
	string actualPose = function.getEncodedKey(target,0);
	newMap.setCurrentPose(target);
	travelledDistance = travelledDistance + target.getDistance(previous);
	string encoding = to_string(target.getX()) + to_string(target.getY());
	visitedCell.emplace(encoding,0);
	
	
	cout << "-----------------------------------------------------------------"<<endl;
	cout << "Round : " << count<< endl;
	newSensedCells = sensedCells + ray.getInformationGain(newMap,x,y,orientation,FOV,range);
	cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;

	ray.performSensingOperation(newMap,x,y,orientation,FOV,range);
	ray.findCandidatePositions(newMap,x,y,orientation,FOV,range);
	vector<pair<long,long> >candidatePosition = ray.getCandidatePositions();
	ray.emptyCandidatePositions();
	
	
	if(candidatePosition.size() == 0) {
	    
	    cout << "No other candidate position" << endl;
	    cout << "----- BACKTRACKING -----" << endl;
	    
	    
	    countBT = countBT -1;
	    if (graph2.size() >0){
		
		// OLD METHOD
		string targetString = graph2.at(countBT).first;
		graph2.pop_back();
		EvaluationRecords record;
		target = record.getPoseFromEncoding(targetString);
		history.push_back(function.getEncodedKey(target,2));
		cout << "[BT]No significative position reachable. Come back to previous position" << endl;
		cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
		count = count + 1;
		cout << "Graph dimension : " << graph2.size() << endl;
		
	    } else {
		cout << "-----------------------------------------------------------------"<<endl;
		cout << "I came back to the original position since i don't have any other candidate position"<< endl;
		cout << "Total cell visited :" << numConfiguration <<endl;
		cout << "FINAL: Map not completely explored!" << endl;
		cout << "-----------------------------------------------------------------"<<endl;
		exit(0);
	    }

	}else{
	    
	    
	    // need to convert from a <int,int pair> to a Pose with also orientation,laser range and angle
	    list<Pose> frontiers;
	    vector<pair<long,long> >::iterator it =candidatePosition.begin();
	    for(it; it != candidatePosition.end(); it++){
		Pose p1 = Pose((*it).first,(*it).second,0 ,range,FOV);
		Pose p2 = Pose((*it).first,(*it).second,180,range,FOV);
		Pose p3 = Pose((*it).first,(*it).second,90,range,FOV);
		Pose p4 = Pose((*it).first,(*it).second,270,range,FOV);
		frontiers.push_back(p1);
		frontiers.push_back(p2);
		frontiers.push_back(p3);
		frontiers.push_back(p4);
	    }
	    
	    unexploredFrontiers = frontiers;
	    
	    cout << "Graph dimension : " << graph2.size() << endl;
	    cout << "Candidate position: " << candidatePosition.size() << endl;
	    cout <<"Frontiers: "<<  frontiers.size() << endl;
	    EvaluationRecords *record = function.evaluateFrontiers(frontiers,newMap,threshold);
	    //cout << "Record: " << record->size() << endl;
	    cout << "Evaluation Record obtained" << endl;
	    
	    
	    if(record->size() != 0){
		//set the previous pose equal to the actual one(actually represented by target)
		previous = target;
		std::pair<string,list<Pose>> pair = make_pair(actualPose,frontiers);
		graph2.push_back(pair);
		std::pair<Pose,double> result = function.selectNewPose(record);
		target = result.first;
		if (!target.isEqual(previous)){
		    count = count + 1;
		    countBT = graph2.size();
		    numConfiguration++;
		    history.push_back(function.getEncodedKey(target,1));
		    cout << "Graph dimension : " << graph2.size() << endl;
		    cout << record->size() << endl;
		}else{
		    cout << "[BT]Cell already explored!Come back to previous position";
		    countBT = countBT -2;
		    string targetString = graph2.at(countBT).first;
		    target = record->getPoseFromEncoding(targetString);
		    graph2.pop_back();
		    history.push_back(function.getEncodedKey(target,2));
		    cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
		    count = count + 1;
		}
	    }else {  
		    
		    //OLD METHOD
		    
		    if(graph2.size() == 0) break;
		    
		    countBT = countBT -1;
		    string targetString = graph2.at(countBT).first;
		    target = record->getPoseFromEncoding(targetString);
		    graph2.pop_back();
		    if(!target.isEqual(previous)){
			previous = target;
			cout << "[BT]No significative position reachable. Come back to previous position" << endl;
			history.push_back(function.getEncodedKey(target,2));
			cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
			count = count + 1;
		    }else {
			countBT = countBT -1;
			string targetString = graph2.at(countBT).first;
			target = record->getPoseFromEncoding(targetString);
			graph2.pop_back();
			previous = target;
			cout << "[BT]No significative position reachable. Come back to previous position" << endl;
			history.push_back(function.getEncodedKey(target,2));
			cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
			count = count + 1;
		    }
		    
	    }
    
	    sensedCells = newSensedCells;
	    //goal.move(target.getX(), target.getY(), cos(target.getOrientation()/2), sin(target.getOrientation()/2));
	    
	    geometry_msgs::PointStamped p;
	    p.header.frame_id = "map";
	    p.header.stamp = ros::Time::now();
	    p.point.x = (target.getX() )* 0.05;
	    p.point.y = (target.getY() )* 0.05;
	    
	    //points.points.push_back(p);
	    //NOTE: not requested for testing purpose
	    //usleep(microseconds);
	    marker_pub.publish(p);
	    //grid_pub.publish< nav_msgs::OccupancyGrid>(grid_msg);
	    sleep(2);
	    frontiers.clear();
	    candidatePosition.clear();
	    delete record;
	    }
	}
    
	newMap.drawVisitedCells(visitedCell,resolution);
	newMap.printVisitedCells(history);


	//OLD METHOD
	if (graph2.size() ==0){
	    cout << "-----------------------------------------------------------------"<<endl;
	    cout << "I came back to the original position since i don't have any other candidate position"<< endl;
	    cout << "Total cell visited :" << numConfiguration <<endl;
	    cout << "-----------------------------------------------------------------"<<endl;
	}else {
	    cout << "-----------------------------------------------------------------"<<endl;
	    cout << "Total cell visited :" << numConfiguration <<endl;
	    cout << "Total travelled distance (cells): " << travelledDistance << endl;
	    cout << "FINAL: MAP EXPLORED!" << endl;
	    cout << "-----------------------------------------------------------------"<<endl;
	}  
	
	return 1;
    }else{
	ROS_ERROR("Failed to call map service");
	return 0;
    }
    
    }


//-----------------------------------------------------------------------
    
    /*
    //MoveBaseGoal goal; 
    ros::Publisher marker_pub = nh.advertise<geometry_msgs::PointStamped>("goal_pt", 10);
    
    //ros::Publisher grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("mcdm_grid", 1000);

    ros::Rate loop_rate(10);
    
    tf::TransformListener listener (ros::Duration(10.0));
    

    
    //ros::spinOnce();
    //loop_rate.sleep();
    //if(cin.get()) break;
    
    ifstream infile;
    infile.open(argv[1]);
    int resolution = atoi(argv[2]);
    //ifstream infile("/home/pulver/Dropbox/Università/Laurea Magistrale/Thesis/testmap10.pgm");
    Map map = Map(infile,resolution);
    cout << "Map dimension: " << map.getNumGridCols() << " : "<<  map.getNumGridRows() << endl;
    // Pose initialPose = map.getRobotPosition();
    
    // i switched x and y because the map's orientation inside and outside programs are different
    long  initX = atoi(argv[4]);	
    long initY = atoi(argv[3]);
    int initOrientation = atoi(argv[5]);
    double initFov = atoi(argv[7] );
    initFov = initFov * PI /180;
    int initRange = atoi(argv[6]);
    double precision = atof(argv[8]);
    double threshold = atof(argv[9]);
    //x,y,orientation,range,angle -
    Pose initialPose = Pose(initX,initY,initOrientation,initRange,initFov);
    Pose target = initialPose;
    Pose previous = initialPose;
    long numConfiguration =0;
    //testing
    vector<pair<string,list<Pose>>> graph2;
    NewRay ray;
    MCDMFunction function;
    long sensedCells = 0;
    long newSensedCells =0;
    long totalFreeCells = map.getTotalFreeCells() ;
    int count = 0;
    int countBT;
    double travelledDistance = 0;
    unordered_map<string,int> visitedCell;
    vector<string>history;
    history.push_back(function.getEncodedKey(target,1));
    //amount of time the robot should do nothing for scanning the environment ( final value expressed in second)
    unsigned int microseconds = 5 * 1000 * 1000 ;
    //cout << "total free cells in the main: " << totalFreeCells << endl;
    list<Pose> unexploredFrontiers;
    
    /*
    nav_msgs::OccupancyGrid grid_msg;
    grid_msg.info.map_load_time = ros::Time::now();
    grid_msg.info.origin.position.x = 0;
    grid_msg.info.origin.position.y = 1;
    grid_msg.info.origin.position.z = 0;
    grid_msg.info.origin.orientation.w = 0;
    grid_msg.info.origin.orientation.x = 0;
    grid_msg.info.origin.orientation.y = 0;
    grid_msg.info.origin.orientation.z = 0;
    grid_msg.info.resolution = 0.05*(100/resolution);
    grid_msg.info.height = map.getNumGridRows();
    grid_msg.info.width = map.getNumGridCols();
    
    for(int n = 0; n < map.getNumGridCols()*map.getNumGridRows(); ++n)
    {
    grid_msg.data.push_back((int8_t)100*map.getGridValue(n));
    }
    */
    
    /*

    while(sensedCells < precision * totalFreeCells ){
	long x = target.getX();
	long y = target.getY();
	int orientation = target.getOrientation();
	int range = target.getRange();
	double FOV = target.getFOV();
	string actualPose = function.getEncodedKey(target,0);
	map.setCurrentPose(target);
	travelledDistance = travelledDistance + target.getDistance(previous);
	string encoding = to_string(target.getX()) + to_string(target.getY());
	visitedCell.emplace(encoding,0);
	
	
	
	
	//Map *map2 = &map;
	
	cout << "-----------------------------------------------------------------"<<endl;
	cout << "Round : " << count<< endl;
	newSensedCells = sensedCells + ray.getInformationGain(map,x,y,orientation,FOV,range);
	cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;

	ray.performSensingOperation(map,x,y,orientation,FOV,range);
	ray.findCandidatePositions(map,x,y,orientation,FOV,range);
	vector<pair<long,long> >candidatePosition = ray.getCandidatePositions();
	ray.emptyCandidatePositions();
	
	
	if(candidatePosition.size() == 0) {
	    
	    cout << "No other candidate position" << endl;
	    cout << "----- BACKTRACKING -----" << endl;
	    
	    
	    countBT = countBT -1;
	    if (graph2.size() >0){
		
		// OLD METHOD
		string targetString = graph2.at(countBT).first;
		graph2.pop_back();
		EvaluationRecords record;
		target = record.getPoseFromEncoding(targetString);
		history.push_back(function.getEncodedKey(target,2));
		cout << "[BT]No significative position reachable. Come back to previous position" << endl;
		cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
		count = count + 1;
		cout << "Graph dimension : " << graph2.size() << endl;
		
	    } else {
		cout << "-----------------------------------------------------------------"<<endl;
		cout << "I came back to the original position since i don't have any other candidate position"<< endl;
		cout << "Total cell visited :" << numConfiguration <<endl;
		cout << "FINAL: Map not completely explored!" << endl;
		cout << "-----------------------------------------------------------------"<<endl;
		exit(0);
	    }

	}else{
	    
	    
	    // need to convert from a <int,int pair> to a Pose with also orientation,laser range and angle
	    list<Pose> frontiers;
	    vector<pair<long,long> >::iterator it =candidatePosition.begin();
	    for(it; it != candidatePosition.end(); it++){
		Pose p1 = Pose((*it).first,(*it).second,0 ,range,FOV);
		Pose p2 = Pose((*it).first,(*it).second,180,range,FOV);
		Pose p3 = Pose((*it).first,(*it).second,90,range,FOV);
		Pose p4 = Pose((*it).first,(*it).second,270,range,FOV);
		frontiers.push_back(p1);
		frontiers.push_back(p2);
		frontiers.push_back(p3);
		frontiers.push_back(p4);
	    }
	    
	    unexploredFrontiers = frontiers;
	    
	    cout << "Graph dimension : " << graph2.size() << endl;
	    cout << "Candidate position: " << candidatePosition.size() << endl;
	    cout <<"Frontiers: "<<  frontiers.size() << endl;
	    EvaluationRecords *record = function.evaluateFrontiers(frontiers,map,threshold);
	    //cout << "Record: " << record->size() << endl;
	    cout << "Evaluation Record obtained" << endl;
	    
	    
	    if(record->size() != 0){
		//set the previous pose equal to the actual one(actually represented by target)
		previous = target;
		std::pair<string,list<Pose>> pair = make_pair(actualPose,frontiers);
		graph2.push_back(pair);
		std::pair<Pose,double> result = function.selectNewPose(record);
		target = result.first;
		if (!target.isEqual(previous)){
		    count = count + 1;
		    countBT = graph2.size();
		    numConfiguration++;
		    history.push_back(function.getEncodedKey(target,1));
		    cout << "Graph dimension : " << graph2.size() << endl;
		    cout << record->size() << endl;
		}else{
		    cout << "[BT]Cell already explored!Come back to previous position";
		    countBT = countBT -2;
		    string targetString = graph2.at(countBT).first;
		    target = record->getPoseFromEncoding(targetString);
		    graph2.pop_back();
		    history.push_back(function.getEncodedKey(target,2));
		    cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
		    count = count + 1;
		}
	    }else {  
		    
		    //OLD METHOD
		    
		    if(graph2.size() == 0) break;
		    
		    countBT = countBT -1;
		    string targetString = graph2.at(countBT).first;
		    target = record->getPoseFromEncoding(targetString);
		    graph2.pop_back();
		    if(!target.isEqual(previous)){
			previous = target;
			cout << "[BT]No significative position reachable. Come back to previous position" << endl;
			history.push_back(function.getEncodedKey(target,2));
			cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
			count = count + 1;
		    }else {
			countBT = countBT -1;
			string targetString = graph2.at(countBT).first;
			target = record->getPoseFromEncoding(targetString);
			graph2.pop_back();
			previous = target;
			cout << "[BT]No significative position reachable. Come back to previous position" << endl;
			history.push_back(function.getEncodedKey(target,2));
			cout << "New target: x = " << target.getY() << ",y = " << target.getX() <<", orientation = " << target.getOrientation() << endl;
			count = count + 1;
		    }
		    
	    }
    
	    sensedCells = newSensedCells;
	    //goal.move(target.getX(), target.getY(), cos(target.getOrientation()/2), sin(target.getOrientation()/2));
	    
	    geometry_msgs::PointStamped p;
	    p.header.frame_id = "map";
	    p.header.stamp = ros::Time::now();
	    p.point.x = (map.getNumGridRows() - target.getX() )* 0.05;
	    p.point.y = (map.getNumGridCols() - target.getY() )* 0.05;
	    
	    //points.points.push_back(p);
	    //NOTE: not requested for testing purpose
	    //usleep(microseconds);
	    marker_pub.publish(p);
	    //grid_pub.publish< nav_msgs::OccupancyGrid>(grid_msg);
	    sleep(2);
	    frontiers.clear();
	    candidatePosition.clear();
	    delete record;
	}
    }
    
    map.drawVisitedCells(visitedCell,resolution);
    map.printVisitedCells(history);


    //OLD METHOD
    if (graph2.size() ==0){
	cout << "-----------------------------------------------------------------"<<endl;
	cout << "I came back to the original position since i don't have any other candidate position"<< endl;
	cout << "Total cell visited :" << numConfiguration <<endl;
	cout << "-----------------------------------------------------------------"<<endl;
    }else {
	cout << "-----------------------------------------------------------------"<<endl;
	cout << "Total cell visited :" << numConfiguration <<endl;
	cout << "Total travelled distance (cells): " << travelledDistance << endl;
	cout << "FINAL: MAP EXPLORED!" << endl;
	cout << "-----------------------------------------------------------------"<<endl;
    }
    
    /*
    
    while(ros::ok())
	sleep(1);

    return 0;
    */
    
    
}
