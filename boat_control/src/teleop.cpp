#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8MultiArray.h"
ros::Publisher speedPub;
int a = false;
void TO_Callback(const std_msgs::Bool::ConstPtr& msg)
{ 
  a = msg->data;
  std_msgs::UInt8MultiArray servoSpeed;
  servoSpeed.data.resize(2) ;
  servoSpeed.data[0] = 1500;
  servoSpeed.data[1] = 1500;  
  if(a)
  {
    ROS_INFO("I got it!");  
    speedPub.publish(servoSpeed);
  }
  a = false;
}

int main(int argc,char**argv)
{
  ros::init(argc,argv,"teleop");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("teleopEnable",1000,TO_Callback);
  speedPub = n.advertise<std_msgs::UInt8MultiArray>("servoState",1000);
  ros::spin();
  return 0;
}
