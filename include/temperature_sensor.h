#include "ros/ros.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"
#include "std_msgs/Header.h"
#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
#include "third_party_sensor_driver.h"

#ifndef TEMPERATURE_SENSOR_
#define TEMPERATURE_SENSOR_

class Temperature_sensor
{
  public:
  Temperature_sensor(ros::NodeHandle nh, const char *port = "ttyUSB0") : serial_port(port)
  {
    std::cout << "Initializing temperature sensor" << std::endl;
    temp_pub = nh.advertise<sensor_msgs::Temperature>("temperature_sensor/temperature", 10);
    pres_pub = nh.advertise<sensor_msgs::FluidPressure>("temperature_sensor/pressure", 10);
    temperature = 0;
    pressure = 0; 
    sensor_initialized = initialize(serial_port);
  }
  void getlatest()
  {
    if (!sensor_initialized) {
       std::cout << "Sensor failed to initialize" << std::endl;
    }

    // pressure data come in as a double
    // no further processing is necessary
    double p = getPressure();

    // Temp data comes as a string
    // some processing is necesary
    std::string t_string = getTemp();
    uint32_t t_int;
    sscanf( t_string.data(), "%x", &t_int);
    double t = (float)t_int;

    uint32_t validity = getValidity();
    // Check Temp validity
    if (validity_checker((uint16_t)(validity >> 16))){
      temperature = t;
    }

    // Check Temp validity
    if (validity_checker((uint16_t)validity)){
      pressure = p;
    }
  }
  void publish()
  { 
    if (!sensor_initialized) {
       std::cout << "Sensor failed to initialize" << std::endl;
    }
    std_msgs::Header header = make_header();
    sensor_msgs::Temperature temp_msg;
    temp_msg.header = header;
    temp_msg.temperature = temperature;

    sensor_msgs::FluidPressure pres_msg;
    pres_msg.header = header; 
    pres_msg.fluid_pressure = pressure;
  
    temp_pub.publish(temp_msg);
    pres_pub.publish(pres_msg);
  }
  
  private:
  ros::Publisher temp_pub;
  ros::Publisher pres_pub;
  bool sensor_initialized;
  std::string serial_port;
  double temperature;
  double pressure;
  bool validity_checker(uint16_t bits)
  {
    uint16_t bits_reverse = (bits >> 8) | (bits << 8);
    return (bits == bits_reverse);
  }
  std_msgs::Header make_header()
  { 
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "temp_sensor";
    return header;
  }
};

#endif //TEMPERATURE_SENSOR_

