/**
Software License Agreement (proprietary)

\file      message_relay_node.cpp
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#include "clock_relay/clock_relay.h"
#include "ros/ros.h"

#include <vector>
#include <string>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "clock_relay_node");

  ros::NodeHandle private_nh("~");

  std::string parameter_namespace;
  if (private_nh.getParam("parameter_namespace", parameter_namespace))
  {
    private_nh = ros::NodeHandle(parameter_namespace);
  }

  std::string from, to;
  if (!private_nh.getParam("from", from) && !private_nh.getParam("to", to))
  {
    ROS_FATAL("Must provide either 'from' and 'to' namespace parameters");
    return 1;
  }

  if (from == to)
  {
    ROS_FATAL("'from' and 'to' namespaces must not be equal");
    return 1;
  }

  ros::NodeHandle origin(from), target(to);

  std::string clock_relay_type;
  if (!private_nh.getParam("clock_relay_type", clock_relay_type))
  {
    ROS_FATAL_STREAM("Must specify if clock relay type is source or sink");
    return 1;
  }

  double throttle_frequency = 0.0;
  std::string key;
  if (private_nh.searchParam("throttle_frequency", key))
  {
    std::string val;
    private_nh.getParam(key, throttle_frequency);
  }

  clock_relay::ClockRelay clock_relay(from, to, clock_relay_type, throttle_frequency);

  ros::spin();

  return 0;
}
