/**
Software License Agreement (proprietary)

\file      frame_id_processor.cpp
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/

#include "message_relay/processor/frame_id_processor.h"

#include "ros/ros.h"

#include <string>

namespace message_relay
{

const boost::unordered_map<std::string, FrameIdProcessor::Operation> FrameIdProcessor::operation_name_map_ =
    boost::assign::map_list_of("", FrameIdProcessor::NONE)("add", FrameIdProcessor::ADD_PREFIX)
        ("selective_add", FrameIdProcessor::SELECTIVE_ADD_PREFIX)("remove", FrameIdProcessor::REMOVE_PREFIX)
        ("selective_remove", FrameIdProcessor::SELECTIVE_REMOVE_PREFIX);

const boost::unordered_map<FrameIdProcessor::Operation, FrameIdProcessor::Operation>
    FrameIdProcessor::operation_inverse_map_ = boost::assign::map_list_of
        (FrameIdProcessor::NONE, FrameIdProcessor::NONE)
        (FrameIdProcessor::REMOVE_PREFIX, FrameIdProcessor::ADD_PREFIX)
        (FrameIdProcessor::SELECTIVE_REMOVE_PREFIX, FrameIdProcessor::SELECTIVE_ADD_PREFIX)
        (FrameIdProcessor::ADD_PREFIX, FrameIdProcessor::REMOVE_PREFIX)
        (FrameIdProcessor::SELECTIVE_ADD_PREFIX, FrameIdProcessor::SELECTIVE_REMOVE_PREFIX);

FrameIdProcessor::ConstPtr FrameIdProcessor::create(std::string tf_prefix, std::string prefix_operation_string,
                                                    boost::unordered_set<std::string> global_frame_names)
{
  try
  {
    FrameIdProcessor::Operation prefix_operation = operation_name_map_.at(prefix_operation_string);
    return FrameIdProcessor::create(tf_prefix, prefix_operation, global_frame_names);
  }
  catch (const std::out_of_range &ex)
  {
    ROS_FATAL_STREAM("Invalid prefix operation " << prefix_operation_string << " specified");
    throw ex;
  }
}

FrameIdProcessor::ConstPtr FrameIdProcessor::create(std::string tf_prefix, FrameIdProcessor::Operation prefix_operation,
                                                    boost::unordered_set<std::string> global_frame_names)
{
  return FrameIdProcessor::ConstPtr(new const FrameIdProcessor(tf_prefix, prefix_operation, global_frame_names));
}

FrameIdProcessor::ConstPtr FrameIdProcessor::inverse(const FrameIdProcessor::ConstPtr &processor)
{
  if (processor)
  {
    return FrameIdProcessor::ConstPtr(new const FrameIdProcessor(processor->tf_prefix_,
        operation_inverse_map_.at(processor->prefix_operation_), processor->global_frame_names_));
  }
  else
  {
    return FrameIdProcessor::ConstPtr();
  }
}

FrameIdProcessor::FrameIdProcessor(std::string tf_prefix, FrameIdProcessor::Operation prefix_operation,
                                   boost::unordered_set<std::string> global_frame_names)
    : tf_prefix_(tf_prefix), prefix_operation_(prefix_operation),
      global_frame_names_(global_frame_names)
{ }

void FrameIdProcessor::process(std::string &frame_id) const
{
  switch (prefix_operation_)
  {
    case FrameIdProcessor::NONE:
      break;

    case FrameIdProcessor::SELECTIVE_ADD_PREFIX:
      ROS_DEBUG_STREAM("Checking for prefix in " << frame_id);
      if (std::mismatch(tf_prefix_.begin(), tf_prefix_.end(), frame_id.begin()).first == tf_prefix_.end())
      {
        ROS_DEBUG_STREAM("frame_id " << frame_id << " already contains prefix " << tf_prefix_);
        break;
      }
      else
      {
        // fall through to ADD_PREFIX
      }
    case FrameIdProcessor::ADD_PREFIX:
      ROS_DEBUG_STREAM("Adding prefix to " << frame_id);
      if (!frame_id.empty() && !global_frame_names_.count(frame_id))
      {
        frame_id = tf_prefix_ + "/" + frame_id;
        ROS_DEBUG_STREAM("Result " << frame_id);
      }
      else
      {
        ROS_DEBUG_STREAM(frame_id << " is a global frame");
      }
      break;

    case FrameIdProcessor::SELECTIVE_REMOVE_PREFIX:
      ROS_DEBUG_STREAM("Checking for prefix in " << frame_id);
      if (std::mismatch(tf_prefix_.begin(), tf_prefix_.end(), frame_id.begin()).first == tf_prefix_.end())
      {
        frame_id = frame_id.substr(tf_prefix_.length() + 1);
        ROS_DEBUG_STREAM("Result " << frame_id);
      }
      else
      {
        ROS_DEBUG_STREAM("frame_id " << frame_id << " doesn't contain prefix " << tf_prefix_);
      }
      break;

    case REMOVE_PREFIX:
      ROS_DEBUG_STREAM("Removing prefix from " << frame_id);
      if (!global_frame_names_.count(frame_id))
      {
        // Check that frame id is at least one character longer than tf_prefix, and that the prefix is followed
        // by a dividing forward slash
        if (frame_id.length() > tf_prefix_.length() + 1 && frame_id[tf_prefix_.length()] == '/')
        {
          frame_id = frame_id.substr(tf_prefix_.length() + 1);
          ROS_DEBUG_STREAM("Result " << frame_id);
        }
        else
        {
          ROS_DEBUG_STREAM("frame_id " << frame_id << " doesn't contain prefix " << tf_prefix_);
        }
      }
      else
      {
        ROS_DEBUG_STREAM(frame_id << " is a global frame");
      }
      break;

    default:
      ROS_ASSERT_MSG(false, "Invalid prefix operation");
  }
}

}  // namespace message_relay
