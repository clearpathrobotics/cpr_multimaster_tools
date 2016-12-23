/**
Software License Agreement (proprietary)

\file      tf2_relay_node.cpp
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#include "tf2_relay/transform_relay.h"
#include "ros/ros.h"

#include <boost/unordered_set.hpp>

#include <vector>
#include <string>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "tf2_relay_node");

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

  std::string tf_prefix, prefix_operation;
  if (private_nh.getParam("tf_prefix", tf_prefix) && !private_nh.getParam("prefix_operation", prefix_operation))
  {
    ROS_FATAL_STREAM("If tf_prefix is provided, a prefix_operation must be specified");
    return 1;
  }

  double throttle_frequency = 0.0;
  std::string key;
  if (private_nh.searchParam("throttle_frequency", key))
  {
    std::string val;
    private_nh.getParam(key, throttle_frequency);
  }

  std::vector<std::string> global_frames;
  private_nh.getParam("global_frames", global_frames);

  message_relay::FrameIdProcessor::ConstPtr frame_id_processor = message_relay::FrameIdProcessor::create(
      tf_prefix, prefix_operation, boost::unordered_set<std::string>(global_frames.begin(), global_frames.end()));

  tf2_relay::TransformRelay transform_relay(origin, target, throttle_frequency, false, frame_id_processor);
  tf2_relay::TransformRelay static_transform_relay(origin, target, throttle_frequency, true, frame_id_processor);

  ros::spin();

  return 0;
}
