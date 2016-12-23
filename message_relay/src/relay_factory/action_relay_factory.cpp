/**
Software License Agreement (proprietary)

\file      action_relay_factory.cpp
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#include "message_relay/relay_factory/action_relay_factory.h"

namespace message_relay
{

ActionRelay::Ptr createActionRelay(const ActionRelayParams &params)
{
  return ActionRelay::Ptr(new ActionRelayImpl(params));
}

}  // namespace message_relay
