#include "ros/ros.h"
#include <cstdio>
#include <iostream>
#include "temperature_sensor.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "temperature_sensor");

  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  std::string serial_port;
  std::string sensor_number;
  nh.getParam(ros::this_node::getName()+"/serial_port", serial_port);
  nh.getParam(ros::this_node::getName()+"/sensor_num", sensor_number);

  Temperature_sensor sensor = Temperature_sensor(nh, sensor_number.c_str(), serial_port.c_str());

  while (ros::ok())
  {
    sensor.getlatest();
    sensor.publish();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
