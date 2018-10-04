#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

serial::Serial gpsport;

int main(int argc,char** argv)
{
  ros::init(argc,argv,"getgps");
  ros::NodeHandle nh;
  ros::Publisher get_data = nh.advertise<std_msgs::String>("gpsdata",1000);
  
  try
  {
    gpsport.setPort("/dev/ttyACM0");
    gpsport.setBaudrate(19200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    gpsport.setTimeout(to);
    gpsport.open();
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
    if(gpsport.available())
    {
      ROS_INFO_STREAM("Reading GPS data");
      std_msgs::String gpsdata;
      gpsdata.data = gpsport.read(gpsport.available());
      ROS_INFO_STREAM("Read:"<<gpsdata.data);
      get_data.publish(gpsdata);
    }
    loop_rate.sleep();
  }
}

