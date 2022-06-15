#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include"geometry_msgs/Twist.h"
int fingers_number;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "msg_handle");
  ros::NodeHandle n;
  void messageCallback(const std_msgs::Int8::ConstPtr& msg);
  ros::Subscriber sub = n.subscribe("gesture_msg", 1000, messageCallback);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("tbmn_01/cmd_vel", 10);
  geometry_msgs::Twist vel_cmd ;
  while(1)
  {
  if(fingers_number==1)   //stop
  {
    vel_cmd.linear.x=0.0;
    vel_cmd.linear.y=0.0;
    vel_cmd.linear.z=0.0;
    vel_cmd.angular.x=0;
    vel_cmd.angular.y=0;
    vel_cmd.angular.z=0;
  }
  else if(fingers_number==2)  //  make turtlesim circle
  {
    vel_cmd.linear.x=2.0;
    vel_cmd.linear.y=0.0;
    vel_cmd.linear.z=0.0;
    vel_cmd.angular.x=0;
    vel_cmd.angular.y=0;
    vel_cmd.angular.z=1.8;
  }
  else if(fingers_number==3)   // go straight
  {
    vel_cmd.linear.x=2.0;
    vel_cmd.linear.y=0.0;
    vel_cmd.linear.z=0.0;
    vel_cmd.angular.x=0;
    vel_cmd.angular.y=0;
    vel_cmd.angular.z=0;
  }
  vel_pub.publish(vel_cmd);
  ros::spinOnce();
  }
  return 0;
}

void messageCallback(const std_msgs::Int8::ConstPtr& msg)
{  
  if(msg->data==1) fingers_number=2;
  else if(msg->data==2) fingers_number=3;
  else if(msg->data==0) fingers_number=1;
}
