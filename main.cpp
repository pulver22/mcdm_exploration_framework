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
geometry_msgs::PoseStamped getCurrentPose();

int main(int argc, char **argv) {
    ros::init(argc, argv, "mcdm_exploration_framework_node");
    ros::NodeHandle nh;
    ros::ServiceClient map_service_client_ = nh.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv_map;
    ros::Publisher grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("mcdm_grid", 1000);
    MoveBaseGoal goal;
    
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
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(origin.position.x, origin.position.y, origin.position.z) );
	//cout << origin.position.x << "," << origin.position.y << endl;
	transform.setRotation(tf::Quaternion(origin.orientation.x,origin.orientation.y,origin.orientation.z,origin.orientation.w));
	//tf::StampedTransform stampTrans(transform,ros::Time::now(),"base_link","base_robot");
	vector<int> data;
	for(int i=0; i < map.data.size(); i++){
	    data.push_back(map.data.at(i));
	}
	Map newMap = Map(resolution,width,height,data,origin);
	
	tf::Transform tranMapToImage;
	tranMapToImage.setOrigin(tf::Vector3(0, 30, 0.0));
	tf::Vector3 vecImageToMap = tf::Vector3(0,30,0.0);
		
	
	// Get the initial pose in map frame
	geometry_msgs::PoseStamped start_pose;
	start_pose = getCurrentPose();
	long initX = start_pose.pose.position.x;
	long initY = start_pose.pose.position.y;
	tf::Quaternion quat = tf::Quaternion(start_pose.pose.orientation.x,start_pose.pose.orientation.y,start_pose.pose.orientation.z,start_pose.pose.orientation.w);
	
	cout <<start_pose.pose.orientation.x <<","<< start_pose.pose.orientation.y <<","<< start_pose.pose.orientation.z<< ","<< start_pose.pose.orientation.w <<endl; 
	
	tfScalar angle = 2 * acos(quat[2]);
	cout << "Initial position in the map frame:" << initX << "," << initY <<" with orientation :" << angle << endl;
	
	int initOrientation = (int)(angle * 57.30);
	
	cout << "Orientation after casting: " << initOrientation << endl;
	double initFov = atoi(argv[1] );
	initFov = initFov * PI /180;
	int initRange = atoi(argv[2]);
	double precision = atof(argv[3]);
	double threshold = atof(argv[4]);
	
	
	//convert from image frame to map
	tf::Vector3 pose = tf::Vector3(initX,initY,0.0);
	pose = tranMapToImage.operator*(pose);
	pose = pose /resolution;
	//pose = transform.operator*(pose);
	
	Pose initialPose = Pose(pose.getY(),pose.getX(),initOrientation,initRange,initFov);
	Pose target = initialPose;
	Pose previous = initialPose;
	
	cout << "Initial position in the image frame: " << target.getY() << "," << target.getX() << endl;
	
	/*
	//----------------- PRINT INITIAL POSITION
	geometry_msgs::PointStamped p;
	p.header.frame_id = "map";
	p.header.stamp = ros::Time::now();
	p.point.x = ( newMap.getNumGridRows() -  target.getX() )* resolution;
	p.point.y = (target.getY() )* resolution;
	
	//cout << p.point.x << ","<< p.point.y << endl;
	
	tf::Vector3 vec =  tf::Vector3(p.point.x,p.point.y,0.0);
	vec = transform.operator*(vec);
	
	p.point.x = vec.getY() ;
	p.point.y = vec.getX() ;
	
	//cout << p.point.x << ","<< p.point.y << endl;
	//----------------------------------------
	*/
	
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
	    
	    //cout << "Graph dimension : " << graph2.size() << endl;
	    //cout << "Candidate position: " << candidatePosition.size() << endl;
	    //cout <<"Frontiers: "<<  frontiers.size() << endl;
	    EvaluationRecords *record = function.evaluateFrontiers(frontiers,newMap,threshold);
	    //cout << "Record: " << record->size() << endl;
	    //cout << "Evaluation Record obtained" << endl;
	    
	    
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
		    //cout << record->size() << endl;
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
	    
	    /*
	    //---------------------------PRINT ACTUAL POSITION
	    geometry_msgs::PointStamped p;
	    p.header.frame_id = "map";
	    p.header.stamp = ros::Time::now();
	    p.point.x = (newMap.getNumGridRows() - target.getX() )* resolution;
	    p.point.y = (target.getY() )* resolution;
	    
	    //cout << p.point.x << ","<< p.point.y << endl;
	    
	    tf::Vector3 vec =  tf::Vector3(p.point.x,p.point.y,0.0);

	    vec = transform.operator*(vec);
	    
	    p.point.x = vec.getY() ;
	    p.point.y = vec.getX() ;
	    
	    //cout << p.point.x << ","<< p.point.y << endl;
	    
	    //NOTE: not requested for testing purpose
	    //usleep(microseconds);
	    marker_pub.publish(p);
	    
	    //--------------------------------------
	    */
	    
	    cout << "Goal in image: " << target.getY() << "," << target.getX() << endl;
	    tf::Vector3 goal_pose = tf::Vector3(target.getY(),target.getX(),0.0);
	    //cout << "1)" <<goal_pose.getX() << "," << goal_pose.getY() << endl;
	    
	    goal_pose = goal_pose * resolution;
	    //cout << "2)" <<goal_pose.getX() << "," << goal_pose.getY() << endl;
	    
	    goal_pose = goal_pose.operator-=(vecImageToMap);
	    //cout << "3)" <<goal_pose.getX() << "," << goal_pose.getY() << endl;
	    
	    cout << "Goal in map: " << goal_pose.getX() <<","<< goal_pose.getY() << endl;
	    goal.move(goal_pose.getX(),goal_pose.getY(), cos(target.getOrientation()/2), sin(target.getOrientation()/2));
	    
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
    
    
}



geometry_msgs::PoseStamped getCurrentPose()
{
    ros::Time _now_stamp_ = ros::Time(0);
   
    tf::StampedTransform start_pose_in_tf;
    tf::TransformListener _tf_listener;
   
    _tf_listener.waitForTransform("map", "base_link", _now_stamp_, ros::Duration(2.0));
    try
    {
        _tf_listener.lookupTransform("map", "base_link", _now_stamp_, start_pose_in_tf);
    }
    catch(tf::TransformException& ex)
    {
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
