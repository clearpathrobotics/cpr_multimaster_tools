/**
Software License Agreement (proprietary)

\file      frame_id_processor.h
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#ifndef MESSAGE_RELAY_PROCESSOR_FRAME_ID_PROCESSOR_H
#define MESSAGE_RELAY_PROCESSOR_FRAME_ID_PROCESSOR_H

#include <string>
#include <vector>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include <boost/assign/list_of.hpp>

namespace message_relay
{

class FrameIdProcessor
{
public:
  typedef boost::shared_ptr<const FrameIdProcessor> ConstPtr;

  enum Operation
  {
    NONE, ADD_PREFIX, SELECTIVE_ADD_PREFIX, REMOVE_PREFIX, SELECTIVE_REMOVE_PREFIX
  };

  static ConstPtr create(std::string tf_prefix, std::string prefix_operation_string,
      boost::unordered_set<std::string> global_frame_names = boost::unordered_set<std::string>());

  static ConstPtr create(std::string tf_prefix, FrameIdProcessor::Operation prefix_operation,
      boost::unordered_set<std::string> global_frame_names = boost::unordered_set<std::string>());

  void process(std::string &frame_id) const;

  static ConstPtr inverse(const ConstPtr &processor);

private:
  FrameIdProcessor(std::string tf_prefix, Operation prefix_operation,
        boost::unordered_set<std::string> global_frame_names);

  static const boost::unordered_map<std::string, Operation> operation_name_map_;
  static const boost::unordered_map<Operation, Operation> operation_inverse_map_;

  std::string tf_prefix_;
  Operation prefix_operation_;
  boost::unordered_set<std::string> global_frame_names_;
};

}  // namespace message_relay

#endif  // MESSAGE_RELAY_PROCESSOR_FRAME_ID_PROCESSOR_H
