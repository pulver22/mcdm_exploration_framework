#include "movebasegoal.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace dummy{
  
MoveBaseGoal::MoveBaseGoal()
:ac ("move_base", true){
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
}

void MoveBaseGoal::move(int x, int y, double orW, double orZ){
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = orW;
    goal.target_pose.pose.orientation.z = orZ;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	ROS_INFO("Hooray, the base moved 1 meter forward");
    else
	ROS_INFO("The base failed to move forward 1 meter for some reason");
}

}