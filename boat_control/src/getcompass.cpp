#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

serial::Serial compassport;

int main(int argc,char** argv)
{
  ros::init(argc,argv,"getcompass");
  ros::NodeHandle nh;
  ros::Publisher get_data = nh.advertise<std_msgs::String>("compassdata",1000);
  
  try
  {
    compassport.setPort("/dev/ttyUSB0");
    compassport.setBaudrate(19200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    compassport.setTimeout(to);
    compassport.open();
  }
  catch(serial::IOException& e)
  { 
    ROS_ERROR_STREAM("Unable to open port");
    return -1;
  }
 
  ros::Rate loop_rate(50);
  while(ros::ok())
  {
    ros::spinOnce();
    if(compassport.available())
    {
      ROS_INFO_STREAM("Reading Compass data");
      std_msgs::String compassdata;
      compassdata.data = compassport.read(compassport.available());
      ROS_INFO_STREAM("Read:"<<compassdata.data);
      get_data.publish(compassdata);
    }
    loop_rate.sleep();
  }
}

