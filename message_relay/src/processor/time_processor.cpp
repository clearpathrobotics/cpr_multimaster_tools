/**
Software License Agreement (proprietary)

\file      time_processor.cpp
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/

#include "message_relay/processor/time_processor.h"

#include <boost/assign/list_of.hpp>
#include <string>

namespace message_relay
{

const boost::unordered_map<std::string, TimeProcessor::Operation> TimeProcessor::operation_name_map_ =
    boost::assign::map_list_of("", TimeProcessor::NONE)("add_offset", TimeProcessor::ADD_OFFSET)
        ("remove_offset", TimeProcessor::REMOVE_OFFSET);

const boost::unordered_map<TimeProcessor::Operation, TimeProcessor::Operation> TimeProcessor::operation_inverse_map_ =
    boost::assign::map_list_of(TimeProcessor::NONE, TimeProcessor::NONE)
        (TimeProcessor::ADD_OFFSET, TimeProcessor::REMOVE_OFFSET)
        (TimeProcessor::REMOVE_OFFSET, TimeProcessor::ADD_OFFSET);

TimeProcessor::ConstPtr TimeProcessor::create(std::string offset_operation_string, ros::Duration offset)
{
  try
  {
    TimeProcessor::Operation offset_operation = operation_name_map_.at(offset_operation_string);
    return TimeProcessor::create(offset_operation, offset);
  }
  catch (const std::out_of_range &ex)
  {
    ROS_FATAL_STREAM("Invalid time offset operation " << offset_operation_string << " specified");
    throw ex;
  }
}

TimeProcessor::ConstPtr TimeProcessor::create(TimeProcessor::Operation offset_operation, ros::Duration offset)
{
  return TimeProcessor::ConstPtr(new const TimeProcessor(offset_operation, offset));
}

TimeProcessor::ConstPtr TimeProcessor::inverse(const TimeProcessor::ConstPtr &processor)
{
  if (processor)
  {
    return TimeProcessor::ConstPtr(new const TimeProcessor(operation_inverse_map_.at(
        processor->offset_operation_), processor->offset_));
  }
  else
  {
    return TimeProcessor::ConstPtr();
  }
}

TimeProcessor::TimeProcessor(Operation offset_operation, ros::Duration offset)
    : offset_operation_(offset_operation), offset_(offset)
{ }

void TimeProcessor::process(ros::Time &time) const
{
  switch (offset_operation_)
  {
    case TimeProcessor::NONE:
      break;

    case TimeProcessor::ADD_OFFSET:
      time += offset_;
      break;

    case TimeProcessor::REMOVE_OFFSET:
      time -= offset_;
      break;

    default:
      ROS_ASSERT_MSG(false, "Invalid time offset operation");
  }
}

}  // namespace message_relay
