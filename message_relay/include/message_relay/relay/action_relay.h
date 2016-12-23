/**
Software License Agreement (proprietary)

\file      action_relay.h
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#ifndef MESSAGE_RELAY_RELAY_ACTION_RELAY_H
#define MESSAGE_RELAY_RELAY_ACTION_RELAY_H

#include "message_relay/relay_factory/topic_relay_factory.h"
#include "message_relay/relay/topic_relay.h"

#include "ros/callback_queue.h"
#include "ros/ros.h"

#include <string>
#include <vector>

namespace message_relay
{

struct ActionRelayParams
{
  std::string type;
  std::string action;
  boost::shared_ptr<ros::NodeHandle> origin;
  boost::shared_ptr<ros::NodeHandle> target;
  FrameIdProcessor::ConstPtr frame_id_processor;
  TimeProcessor::ConstPtr time_processor;
  double feedback_throttle_frequency;
  int queue_size;
  boost::shared_ptr<ros::CallbackQueue> callback_queue;

  ActionRelayParams()
      : queue_size(100)
  { }
};

class ActionRelay
{
public:
  typedef boost::shared_ptr<ActionRelay> Ptr;

  virtual ~ActionRelay()
  { }

protected:
  ActionRelay()
  { }
};

class ActionRelayImpl : public ActionRelay
{
  friend ActionRelay::Ptr createActionRelay(const ActionRelayParams &params);

private:
  explicit ActionRelayImpl(const ActionRelayParams &params)
  {
    message_relay::TopicRelayParams status_params;
    status_params.queue_size = params.queue_size;
    status_params.callback_queue = params.callback_queue;
    status_params.origin = params.origin;
    status_params.target = params.target;
    status_params.frame_id_processor = params.frame_id_processor;
    status_params.time_processor = params.time_processor;
    status_params.type = "actionlib_msgs/GoalStatusArray";
    status_params.topic = params.action + "/status";
    relays_.push_back(message_relay::createTopicRelay(status_params));

    message_relay::TopicRelayParams feedback_params;
    feedback_params.throttle_frequency = params.feedback_throttle_frequency;
    feedback_params.queue_size = params.queue_size;
    feedback_params.callback_queue = params.callback_queue;
    feedback_params.origin = params.origin;
    feedback_params.target = params.target;
    feedback_params.frame_id_processor = params.frame_id_processor;
    status_params.time_processor = params.time_processor;
    feedback_params.type = params.type + "Feedback";
    feedback_params.topic = params.action + "/feedback";
    relays_.push_back(message_relay::createTopicRelay(feedback_params));

    message_relay::TopicRelayParams result_params;
    result_params.queue_size = params.queue_size;
    result_params.callback_queue = params.callback_queue;
    result_params.origin = params.origin;
    result_params.target = params.target;
    result_params.frame_id_processor = params.frame_id_processor;
    status_params.time_processor = params.time_processor;
    result_params.type = params.type + "Result";
    result_params.topic = params.action + "/result";
    relays_.push_back(message_relay::createTopicRelay(result_params));

    message_relay::TopicRelayParams goal_params;
    goal_params.queue_size = params.queue_size;
    goal_params.callback_queue = params.callback_queue;
    goal_params.origin = params.target;
    goal_params.target = params.origin;
    goal_params.frame_id_processor = FrameIdProcessor::inverse(params.frame_id_processor);
    status_params.time_processor = TimeProcessor::inverse(params.time_processor);
    goal_params.type = params.type + "Goal";
    goal_params.topic = params.action + "/goal";
    relays_.push_back(message_relay::createTopicRelay(goal_params));

    message_relay::TopicRelayParams cancel_params;
    cancel_params.queue_size = params.queue_size;
    cancel_params.callback_queue = params.callback_queue;
    cancel_params.origin = params.target;
    cancel_params.target = params.origin;
    goal_params.frame_id_processor = FrameIdProcessor::inverse(params.frame_id_processor);
    status_params.time_processor = TimeProcessor::inverse(params.time_processor);
    cancel_params.type = "actionlib_msgs/GoalID";
    cancel_params.topic = params.action + "/cancel";
    relays_.push_back(message_relay::createTopicRelay(cancel_params));
  }

  std::vector<message_relay::TopicRelay::Ptr> relays_;
};

}  // namespace message_relay

#endif  // MESSAGE_RELAY_RELAY_ACTION_RELAY_H
