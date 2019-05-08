/**

Connects to move base and rviz to test path lenghts from make plan service

Uses clicked point to define start and end points.

 */



#include <ros/ros.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

#include "nav_msgs/GetPlan.h"

#include <fstream>      // std::ofstream
#include <limits>

using namespace ros;
using namespace std;

int storedPoints;
nav_msgs::GetPlan path;

void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  if (storedPoints==0)
  {
    storedPoints++;
    path.request.start.header.frame_id=msg->header.frame_id;
    path.request.start.pose.position.x = msg->point.x;
    path.request.start.pose.position.y = msg->point.y;
    path.request.start.pose.orientation.w = 1;
  } else if (storedPoints==1)
  {
    storedPoints++;
    path.request.goal.header.frame_id=msg->header.frame_id;
    path.request.goal.pose.position.x = msg->point.x;
    path.request.goal.pose.position.y = msg->point.y;
    path.request.goal.pose.orientation.w = 1;
  }
  ROS_INFO("I have [%d] points stored",storedPoints);
}

double getPathLen(std::vector<geometry_msgs::PoseStamped> poses)
{
  double len=0;
  geometry_msgs::Point p1, p2;
  int npoints = poses.size();
  ROS_INFO("Path has [%d] points",npoints);
  if(npoints>0){
      for (int i=1;i<npoints;i++){
          p1=poses[i].pose.position;
          p2=poses[i-1].pose.position;
          len += sqrt(pow(p1.x - p2.x, 2) +   pow(p1.y - p2.y, 2) );
      }
  } else {
    len=std::numeric_limits<double>::max();
    ROS_INFO("Empty path. Len set to infinite... ");
  }

  return len;
}

int main(int argc,char **argv){
  double len;
  ros::Time ping,pong,loop_pong,loop_ping;
  bool srv_call;

  ros::init(argc, argv, "test");
  ros::NodeHandle node;

  ros::Rate r(2);
  path.request.tolerance = 0.5;

  storedPoints=0;

  // connect to move_base getPlan
  ping = ros::Time::now();
  ros::ServiceClient client = node.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan",true);
  pong = ros::Time::now();
  // time it!
  ROS_INFO("connect to service lasted [%3.3f msecs]",(pong-ping).toNSec()/1e6);

  // get poses easily in rviz
  ping = ros::Time::now();
  ros::Subscriber sub = node.subscribe("/clicked_point", 1000, pointCallback);
  pong = ros::Time::now();
  // time it!
  ROS_INFO("create subscriber took [%3.3f msecs]",(pong-ping).toNSec()/1e6);

  while (ros::ok())
  {
      loop_ping = ros::Time::now();
      if (storedPoints==2){

          storedPoints=0;
          // get a path
          ping = ros::Time::now();
          srv_call = client.call(path);
          pong = ros::Time::now();
          // time it!
          ROS_INFO("Service call  took [%3.3f msecs]",(pong-ping).toNSec()/1e6);


          if(srv_call){
              // calculate path lenght
              ping = ros::Time::now();
              len = getPathLen(path.response.plan.poses);
              // time it!
              pong = ros::Time::now();
              ROS_INFO("Path len calculation took [%3.3f msecs]",(pong-ping).toNSec()/1e6);
              if (len<1e3)
              {
                ROS_INFO("Path len is [%3.3f m.]",len);
              }
              else
              {
                ROS_INFO("Path len is infinite");
              }
        	} else {
            ROS_INFO("Service call failed! ");
          }
      }
      ros::spinOnce();
      r.sleep();
      loop_pong = ros::Time::now();
      ROS_INFO_THROTTLE(60,"Control loop took [%3.3f msecs]",(loop_pong-loop_ping).toNSec()/1e6);
    }

return 0;

}
