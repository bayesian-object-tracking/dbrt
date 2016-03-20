/*
 * This is part of the Bayesian Object Tracking (bot),
 * (https://github.com/bayesian-object-tracking)
 *
 * Copyright (c) 2015 Max Planck Society,
 * 				 Autonomous Motion Department,
 * 			     Institute for Intelligent Systems
 *
 * This Source Code Form is subject to the terms of the GNU General Public
 * License License (GNU GPL). A copy of the license can be found in the LICENSE
 * file distributed with this source code.
 */

/**
 * \file fusion_tracker_node.hpp
 * \date Januray 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <fl/util/profiling.hpp>

#include <dbrt/fusion_tracker_node.h>

#include <dbot_ros/utils/ros_interface.hpp>

namespace dbrt
{
template <typename Tracker>
FusionTrackerNode<Tracker>::FusionTrackerNode(
    const std::shared_ptr<Tracker>& tracker,
    const std::shared_ptr<dbot::RobotPublisher<State>>& publisher)
    : tracker_(tracker), publisher_(publisher)
{
}

template <typename Tracker>
void FusionTrackerNode<Tracker>::tracking_callback(
    const sensor_msgs::Image& ros_image,
    const JointsObsrv& joint_obsrv)
{
    auto image = ri::Ros2EigenVector<typename Obsrv::Scalar>(
        ros_image, tracker_->camera_data()->downsampling_factor());

    current_state_ = tracker_->track(image, joint_obsrv);
    publisher_->publish(current_state_, ros_image, tracker_->camera_data());
}

template <typename Tracker>
auto FusionTrackerNode<Tracker>::current_state() const -> const State &
{
    return current_state_;
}
}
