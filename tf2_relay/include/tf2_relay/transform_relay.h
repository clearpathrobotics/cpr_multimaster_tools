/**
Software License Agreement (proprietary)

\file      transform_relay.h
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#ifndef TF2_RELAY_TRANSFORM_RELAY_H
#define TF2_RELAY_TRANSFORM_RELAY_H

#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"

#include "message_relay/processor/frame_id_processor.h"

#include <boost/unordered_map.hpp>

#include <string>
#include <utility>

namespace tf2_relay
{

class TransformRelay
{
public:
  TransformRelay(ros::NodeHandle origin, ros::NodeHandle target, double frequency, bool is_static,
                 message_relay::FrameIdProcessor::ConstPtr frame_id_processor);

private:
  void transformCb(const tf2_msgs::TFMessageConstPtr &transforms);

  void relayCb();

  typedef std::pair<std::string, std::string> FrameIdPair;

  void processTransform(const geometry_msgs::TransformStamped &new_tf);

  ros::NodeHandle origin_, target_;
  ros::Timer relay_timer_;
  message_relay::FrameIdProcessor::ConstPtr frame_id_processor_;

  ros::Publisher tf_publisher_;
  ros::Subscriber tf_subscriber_;

  tf2_msgs::TFMessage transform_cache_;
  boost::unordered_map<FrameIdPair, std::size_t> transform_cache_index_map_;
};

}  // namespace tf2_relay

#endif  // TF2_RELAY_TRANSFORM_RELAY_H
