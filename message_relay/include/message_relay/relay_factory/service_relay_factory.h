/**
Software License Agreement (proprietary)

\file      service_relay_factory.h
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#ifndef MESSAGE_RELAY_RELAY_FACTORY_SERVICE_RELAY_FACTORY_H
#define MESSAGE_RELAY_RELAY_FACTORY_SERVICE_RELAY_FACTORY_H

#include "message_relay/relay/service_relay.h"

namespace message_relay
{

ServiceRelay::Ptr createServiceRelay(const ServiceRelayParams &params);

}  // namespace message_relay

#endif  // MESSAGE_RELAY_RELAY_FACTORY_SERVICE_RELAY_FACTORY_H
