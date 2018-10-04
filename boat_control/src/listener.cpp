#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Int8.h"
void DS_Callback(const std_msgs::String::ConstPtr& msg);
void SSR_Callback(const std_msgs::Int8::ConstPtr& msg);
void SSL_Callback(const std_msgs::Int8::ConstPtr& msg);
ros::ServiceClient StartArduino;
ros::Publisher StartTeleop;
int main(int argc,char**argv)
{
   ros::init(argc,argv,"listener");
   ros::NodeHandle n;
   StartArduino = n.serviceClient<std_srvs::SetBool>("arduinoEnable");
   ros::Subscriber drivesub = n.subscribe("driveStatus",1000,DS_Callback);
   ros::Subscriber speedRsub = n.subscribe("speedStatusR",1000,SSR_Callback);
   ros::Subscriber speedLsub = n.subscribe("speedStatusL",1000,SSL_Callback);
   StartTeleop = n.advertise<std_msgs::Bool>("teleopEnable",1000);
   std_srvs::SetBool enable;
   enable.request.data = true;
   while(!(StartArduino.call(enable)));
   ROS_INFO("Arduino starts up");
   ros::spin();
   return 0;
}

void DS_Callback(const std_msgs::String::ConstPtr& msg)
{
   std_msgs::Bool StartTO;
   ROS_INFO("I heard:[%s]",msg->data.c_str());
   if(msg->data == "teleoperate")
   {
      StartTO.data = true;
      StartTeleop.publish(StartTO);
   }

}

void SSR_Callback(const std_msgs::Int8::ConstPtr& msg)
{
   ROS_INFO("Right:[%d]",msg->data);
}

void SSL_Callback(const std_msgs::Int8::ConstPtr& msg)
{
   ROS_INFO("Left:[%d]",msg->data);
}
