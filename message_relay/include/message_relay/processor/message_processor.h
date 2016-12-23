/**
Software License Agreement (proprietary)

\file      message_processor.h
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#ifndef MESSAGE_RELAY_PROCESSOR_MESSAGE_PROCESSOR_H
#define MESSAGE_RELAY_PROCESSOR_MESSAGE_PROCESSOR_H

#include "message_relay/processor/frame_id_processor.h"
#include "message_relay/processor/time_processor.h"

namespace message_relay
{

template<typename Message, typename Processor>
class MessageProcessor
{
public:
  static void processMessage(typename Message::Ptr &msg, typename Processor::ConstPtr &processor);
};

template<typename Service, typename Processor>
class ServiceProcessor
{
public:
  static void processRequest(typename Service::Request &req, typename Processor::ConstPtr &processor);

  static void processResponse(typename Service::Response &res, typename Processor::ConstPtr &processor);
};

}  // namespace message_relay

#endif  // MESSAGE_RELAY_PROCESSOR_MESSAGE_PROCESSOR_H
