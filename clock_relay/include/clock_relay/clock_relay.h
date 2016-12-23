/**
Software License Agreement (proprietary)

\file      clock_relay.h
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#ifndef CLOCK_RELAY_CLOCK_RELAY_H
#define CLOCK_RELAY_CLOCK_RELAY_H

#include "message_relay/relay_factory/topic_relay_factory.h"
#include "multimaster_msgs/GetClockOffset.h"
#include "multimaster_msgs/ClockOffset.h"

#include "ros/ros.h"

#include <string>

namespace clock_relay
{

class ClockRelay
{
enum Type
{
  SOURCE, SINK
};

public:
  ClockRelay(std::string from, std::string to, std::string clock_relay_type_string, double frequency);

private:
  bool getClockOffsetCb(multimaster_msgs::GetClockOffset::Request &req,
      multimaster_msgs::GetClockOffset::Response &res);

  multimaster_msgs::ClockOffset offset_;
  ros::ServiceServer offset_server_;

  message_relay::TopicRelay::Ptr clock_relay_;
  ros::Publisher offset_publisher_;

  static const boost::unordered_map<std::string, Type> type_name_map_;
};

}  // namespace clock_relay

#endif  // CLOCK_RELAY_CLOCK_RELAY_H
