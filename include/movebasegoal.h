#ifndef MOVEBASEGOAL_H_
#define MOVEBASEGOAL_H_

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

namespace dummy {

class MoveBaseGoal {
  MoveBaseClient ac;

public:
  MoveBaseGoal();

  void move(int x, int y, double orW, double orZ);
};
}

#endif