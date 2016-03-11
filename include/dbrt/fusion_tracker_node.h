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
 * \file tracker_node.h
 * \date Januray 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <memory>
#include <dbot_ros/tracker_publisher.h>

namespace dbrt
{
/**
 * \brief Represents a generic tracker node
 */
template <typename Tracker>
class FusionTrackerNode
{
public:
    typedef typename Tracker::State State;
    typedef typename Tracker::Obsrv Obsrv;
    typedef typename Tracker::JointObsrv JointObsrv;
    typedef typename Tracker::Obsrv JointsObsrv;

public:
    /**
     * \brief Creates a TrackerNode
     */
    FusionTrackerNode(
        const std::shared_ptr<Tracker>& tracker,
        const std::shared_ptr<dbot::TrackerPublisher<State>>& publisher);

    /**
     * \brief Tracking callback function which is invoked whenever a new image
     *        is available
     */
    void tracking_callback(const sensor_msgs::Image& ros_image,
                           const JointsObsrv &joint_obsrv);

    const State& current_state() const;

protected:
    State current_state_;
    std::shared_ptr<Tracker> tracker_;
    std::shared_ptr<dbot::TrackerPublisher<State>> publisher_;
};
}
