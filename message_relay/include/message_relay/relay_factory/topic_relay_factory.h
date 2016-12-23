/**
Software License Agreement (proprietary)

\file      topic_relay_factory.h
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#ifndef MESSAGE_RELAY_RELAY_FACTORY_TOPIC_RELAY_FACTORY_H
#define MESSAGE_RELAY_RELAY_FACTORY_TOPIC_RELAY_FACTORY_H

#include "message_relay/relay/topic_relay.h"

namespace message_relay
{

TopicRelay::Ptr createTopicRelay(const TopicRelayParams &params);

}  // namespace message_relay

#endif  // MESSAGE_RELAY_RELAY_FACTORY_TOPIC_RELAY_FACTORY_H
