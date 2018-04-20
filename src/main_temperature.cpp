#include "ros/ros.h"
#include <cstdio>
#include <iostream>
#include "temperature_sensor.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "temperature_sensor");

  ros::NodeHandle temp_node_handle;

  ros::Rate loop_rate(10);

  Temperature_sensor sensor = Temperature_sensor(temp_node_handle, "serialportname");

  while (ros::ok())
  {
    sensor.getlatest();
    sensor.publish();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
