#ifndef MOVEBASEGOAL_H_
#define MOVEBASEGOAL_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace dummy{
  
class MoveBaseGoal{
   MoveBaseClient ac;

public:
  
MoveBaseGoal();

void move(int x, int y, double orW, double orZ);
    
};

}

#endif