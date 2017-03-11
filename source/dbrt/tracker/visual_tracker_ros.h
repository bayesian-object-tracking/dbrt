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
 * \file ros_robot_tracker.h
 * \date June 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <memory>
#include <mutex>
#include <dbrt/tracker/visual_tracker.h>
#include <ros/time.h>

namespace dbrt
{
/**
 * \brief Represents a generic tracker node
 */
class VisualTrackerRos
{
public:
    typedef VisualTracker::State State;
    typedef VisualTracker::Obsrv Obsrv;

public:
    /**
     * \brief Creates a VisualTrackerRos
     */
    VisualTrackerRos(const std::shared_ptr<VisualTracker>& tracker,
                const std::shared_ptr<dbot::CameraData>& camera_data);

    /**
     * \brief Tracking callback function which is invoked whenever a new image
     *        is available
     */
    void track(const sensor_msgs::Image& ros_image);
    void initialize(const std::vector<State>& initial_states);
    void run();
    bool process();
    void shutdown();

    /**
     * \brief Incoming observation callback function
     * \param ros_image new observation
     */
    void update_obsrv(const sensor_msgs::Image& ros_image);

    void get_current_state(State& state, ros::Time& time) const;
    const std::shared_ptr<VisualTracker>& tracker() { return tracker_; }

protected:
    bool obsrv_updated_;
    bool running_;
    State current_state_;
    ros::Time current_time_;
    sensor_msgs::Image current_ros_image_;
    std::mutex obsrv_mutex_;
    std::mutex state_mutex_;
    std::shared_ptr<VisualTracker> tracker_;
    std::shared_ptr<dbot::CameraData> camera_data_;
};

}
