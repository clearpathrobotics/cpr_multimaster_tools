/**
Software License Agreement (proprietary)

\file      transform_relay.cpp
\authors   Paul Bovbel <pbovbel@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#include "tf2_relay/transform_relay.h"

#include "message_relay/processor/message_processor.h"

#include <utility>

namespace tf2_relay
{

TransformRelay::TransformRelay(ros::NodeHandle origin, ros::NodeHandle target, double frequency, bool is_static,
                               message_relay::FrameIdProcessor::ConstPtr frame_id_processor)
    : origin_(origin), target_(target), frame_id_processor_(frame_id_processor)
{
  if (!is_static)
  {
    tf_subscriber_ = origin_.subscribe<tf2_msgs::TFMessage>
        ("tf", 100, boost::bind(&TransformRelay::transformCb, this, _1));
    tf_publisher_ = target_.advertise<tf2_msgs::TFMessage>("tf", 100);
  }
  else
  {
    tf_subscriber_ = origin_.subscribe<tf2_msgs::TFMessage>
        ("tf_static", 100, boost::bind(&TransformRelay::transformCb, this, _1));
    tf_publisher_ = target_.advertise<tf2_msgs::TFMessage>("tf_static", 100, true);
  }

  if (!is_static && frequency > 0.0)
  {
    relay_timer_ = origin_.createTimer(ros::Duration(1 / frequency), boost::bind(&TransformRelay::relayCb, this));
  }
}

void TransformRelay::transformCb(const tf2_msgs::TFMessageConstPtr &transforms)
{
  for (tf2_msgs::TFMessage::_transforms_type::const_iterator new_tf = transforms->transforms.begin();
       new_tf != transforms->transforms.end(); ++new_tf)
  {
    processTransform(*new_tf);
  }

  // If no timer is setup, publish immediately
  if (!relay_timer_.isValid() && !transform_cache_.transforms.empty())
  {
    tf_publisher_.publish(transform_cache_);
  }
}

void TransformRelay::relayCb()
{
  if (!transform_cache_.transforms.empty())
  {
    tf_publisher_.publish(transform_cache_);
  }

  transform_cache_.transforms.clear();
  transform_cache_index_map_.clear();
}

void TransformRelay::processTransform(const geometry_msgs::TransformStamped &new_tf)
{
  const FrameIdPair frame_id_pair(new_tf.header.frame_id, new_tf.child_frame_id);

  // Check if incoming transform needs to be added to transform cache
  if (transform_cache_index_map_.count(frame_id_pair) == 0)
  {
    geometry_msgs::TransformStamped::Ptr new_tf_ptr = boost::make_shared<geometry_msgs::TransformStamped>(new_tf);
    message_relay::MessageProcessor<geometry_msgs::TransformStamped, message_relay::FrameIdProcessor>::processMessage(
        new_tf_ptr, frame_id_processor_);
    transform_cache_.transforms.push_back(*new_tf_ptr);
    // Cache the index of the just-added transform
    transform_cache_index_map_.insert(
        std::pair<FrameIdPair, std::size_t>(frame_id_pair, transform_cache_.transforms.size() - 1));
  }
    // Otherwise update the existing transform in cache
  else
  {
    geometry_msgs::TransformStamped &tf =
        transform_cache_.transforms[transform_cache_index_map_[frame_id_pair]];

    // Only update if new transform is newer
    if (tf.header.stamp < new_tf.header.stamp)
    {
      tf.header.stamp = new_tf.header.stamp;
      tf.transform = new_tf.transform;
    }
  }
}

}  // namespace tf2_relay
