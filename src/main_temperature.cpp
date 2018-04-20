#include "ros/ros.h"
#include "sensor_msgs/Temperature.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "temperature_sensor");

  ros::NodeHandle n;

  ros::Publisher temp_pub = n.advertise<sensor_msgs::Temperature>("temperature_sensor/data", 10);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    sensor_msgs::Temperature msg;

    temp_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
