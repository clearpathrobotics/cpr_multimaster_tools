/**
Software License Agreement (proprietary)

\file      time_processor.h
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#ifndef MESSAGE_RELAY_PROCESSOR_TIME_PROCESSOR_H
#define MESSAGE_RELAY_PROCESSOR_TIME_PROCESSOR_H

#include "ros/ros.h"

#include <boost/unordered_map.hpp>
#include <string>
#include <vector>

namespace message_relay
{

class TimeProcessor
{
public:
  typedef boost::shared_ptr<const TimeProcessor> ConstPtr;

  enum Operation
  {
    NONE, ADD_OFFSET, REMOVE_OFFSET
  };

  static ConstPtr create(std::string offset_operation_string, ros::Duration offset = ros::Duration(0.0));

  static ConstPtr create(Operation offset_operation, ros::Duration offset = ros::Duration(0.0));

  void process(ros::Time &time) const;

  static ConstPtr inverse(const ConstPtr &processor);

private:
  TimeProcessor(Operation offset_operation, ros::Duration offset);

  static const boost::unordered_map<std::string, Operation> operation_name_map_;
  static const boost::unordered_map<Operation, Operation> operation_inverse_map_;

  Operation offset_operation_;
  ros::Duration offset_;
};

}  // namespace message_relay

#endif  // MESSAGE_RELAY_PROCESSOR_TIME_PROCESSOR_H
