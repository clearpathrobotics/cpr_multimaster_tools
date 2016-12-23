/**
Software License Agreement (proprietary)

\file      message_relay_node.cpp
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#include "clock_relay/clock_relay.h"

#include "message_relay/processor/message_processor.h"

#include <string>

namespace clock_relay
{

const boost::unordered_map<std::string, ClockRelay::Type> ClockRelay::type_name_map_ =
    boost::assign::map_list_of("source", ClockRelay::SOURCE)("sink", ClockRelay::SINK);

ClockRelay::ClockRelay(std::string from, std::string to, std::string clock_relay_type_string, double frequency)
{
  ClockRelay::Type clock_relay_type;
  try
  {
    clock_relay_type = type_name_map_.at(clock_relay_type_string);
  }
  catch (const std::out_of_range &ex)
  {
    ROS_FATAL_STREAM("Invalid clock relay type " << clock_relay_type_string << " specified");
    throw ex;
  }

  message_relay::TopicRelayParams params;
  params.topic = "clock";
  params.type = "rosgraph_msgs/Clock";
  params.origin = boost::make_shared<ros::NodeHandle>(from);
  params.target = boost::make_shared<ros::NodeHandle>(to);
  params.queue_size = 1;
  params.throttle_frequency = frequency;

  switch (clock_relay_type)
  {
    case ClockRelay::SOURCE:
    {
      // Synchronize offset to current walltime
      ros::WallTime now(ros::WallTime::now());

      // Publish offset on 'clock_offset' topic
      offset_publisher_ = params.origin->advertise<multimaster_msgs::ClockOffset>("/clock_offset", 1, true);
      offset_.offset.fromNSec(now.toNSec());
      offset_publisher_.publish(offset_);

      offset_server_ = params.origin->advertiseService("/get_clock_offset", &ClockRelay::getClockOffsetCb, this);

      // offset outgoing clock
      params.time_processor = message_relay::TimeProcessor::create(
          message_relay::TimeProcessor::ADD_OFFSET, offset_.offset);
      break;
    }
    case ClockRelay::SINK:
    {
      // No processing needed
      break;
    }
    default:
    {
      ROS_FATAL_STREAM("Unsupported clock relay type " << clock_relay_type_string << " specified");
      break;
    }
  }

  clock_relay_ = message_relay::createTopicRelay(params);
}

bool ClockRelay::getClockOffsetCb(multimaster_msgs::GetClockOffset::Request &req,
    multimaster_msgs::GetClockOffset::Response &res)
{
  res.clock_offset = offset_;
  return true;
}

}  // namespace clock_relay
