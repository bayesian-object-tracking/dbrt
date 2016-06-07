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
 * \file visual_tracker.hpp
 * \date June 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <dbrt/ros_visual_tracker.h>

#include <dbot_ros/utils/ros_interface.hpp>

namespace dbrt
{
RosVisualTracker::RosVisualTracker(
                                   const std::shared_ptr<VisualTracker>& tracker,
    const std::shared_ptr<dbot::CameraData>& camera_data)
    : tracker_(tracker),
      camera_data_(camera_data),
      obsrv_updated_(false),
      running_(false)
{
}

void RosVisualTracker::track(const sensor_msgs::Image& ros_image)
{
    auto image = ri::to_eigen_vector<typename Obsrv::Scalar>(
        ros_image, camera_data_->downsampling_factor());

    current_state_ = tracker_->track(image);
    // current_pose_.pose = ri::to_ros_pose(current_state_);
    // current_pose_.header.stamp = ros_image.header.stamp; 
    // current_pose_.header.frame_id= ros_image.header.frame_id;
}


void RosVisualTracker::update_obsrv(
    const sensor_msgs::Image& ros_image)
{
    std::lock_guard<std::mutex> lock_obsrv(obsrv_mutex_);
    current_ros_image_ = ros_image;
    obsrv_updated_     = true;
}


void RosVisualTracker::shutdown()
{
    running_ = false;
}

void RosVisualTracker::run()
{
    running_ = true;

    while (ros::ok() && running_)
    {
        if (!obsrv_updated_)
        {
            usleep(100);
            continue;
        }

        process();
    }
}

bool RosVisualTracker::process()
{
    if (!obsrv_updated_) return false;

    Obsrv obsrv;
    sensor_msgs::Image ros_image;
    {
        std::lock_guard<std::mutex> lock_obsrv(obsrv_mutex_);
        ros_image      = current_ros_image_;
        obsrv_updated_ = false;
    }
    track(ros_image);

    return true;
}

auto RosVisualTracker::current_state() const -> const State &
{
    return current_state_;
}

}
