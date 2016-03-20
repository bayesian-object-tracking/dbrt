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
 * \file fusion_tracker.cpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <ros/ros.h>
#include <dbrt/fusion_tracker.h>

#include <dbot_ros/utils/ros_interface.hpp>

namespace dbrt
{
FusionTracker::FusionTracker(
    const std::shared_ptr<RotaryTracker>&
        gaussian_joint_tracker,
    const std::shared_ptr<VisualTracker>&
        rbc_particle_filter_tracker)
    : gaussian_joint_tracker_(gaussian_joint_tracker),
      rbc_particle_filter_tracker_(rbc_particle_filter_tracker),
      running_(true)
{
}

void FusionTracker::initialize(const std::vector<State>& initial_states)
{
    current_state_ = initial_states[0];
    gaussian_joint_tracker_->initialize(initial_states);
    rbc_particle_filter_tracker_->initialize(initial_states);
}

void FusionTracker::run_gaussian_tracker()
{
    while (running_)
    {
        usleep(1);
        std::deque<JointsObsrvEntry> joints_obsrvs_buffer_copy;
        {
            std::lock_guard<std::mutex> lock(joints_obsrv_buffer_mutex_);
            joints_obsrvs_buffer_copy = joints_obsrvs_buffer_;
            joints_obsrvs_buffer_.clear();
        }

        State current_state;
        {
            std::lock_guard<std::mutex> state_lock(current_state_mutex_);
            current_state = current_state_;
        }

        for (auto joints_obsrv_entry : joints_obsrvs_buffer_copy)
        {
            // construct a joints belief entry which contains the following
            //  - the joints measurement values
            //  - their time stamp
            //  - the updated joints belief
            JointsBeliefEntry joints_belief_entry;
            joints_belief_entry.joints_obsrv_entry = joints_obsrv_entry;

            // store current belief
            current_state = update_with_joints(
                joints_belief_entry.joints_obsrv_entry.obsrv);

            joints_belief_entry.beliefs = gaussian_joint_tracker_->beliefs();

            // update sliding window of belief and joints obsrv entries
            joints_obsrv_belief_buffer_.push_back(joints_belief_entry);
            if (joints_obsrv_belief_buffer_.size() > 10000)
            {
                joints_obsrv_belief_buffer_.pop_front();
            }
        }

        {
            std::lock_guard<std::mutex> state_lock(current_state_mutex_);
            //            current_state_ = current_state;
        }
    }
}

void FusionTracker::run_particle_tracker()
{
    while (running_)
    {
        // here is where the magic happens

//        usleep(33000);
////        // obtain latest image obsrv copy
//        sensor_msgs::Image ros_image;
//        {
//            std::lock_guard<std::mutex> lock(image_obsrvs_mutex_);
//            ros_image = ros_image_;
//        }

//        if (ros_image.height == 0 || ros_image.width == 0) continue;

//        auto image = ri::Ros2EigenVector<double>(
//            ros_image,
//            rbc_particle_filter_tracker_->camera_data()->downsampling_factor());

//        State current_state;
//        current_state = rbc_particle_filter_tracker_->track(image);

//        {
//            std::lock_guard<std::mutex> state_lock(current_state_mutex_);
//            current_state_ = current_state;
//        }
    }
}

void FusionTracker::run()
{
    running_ = true;
    gaussian_tracker_thread_ =
        std::thread(&FusionTracker::run_gaussian_tracker, this);
    particle_tracker_thread_ =
        std::thread(&FusionTracker::run_particle_tracker, this);
}

void FusionTracker::shutdown()
{
    running_ = false;
    gaussian_tracker_thread_.join();
    particle_tracker_thread_.join();
}

FusionTracker::State FusionTracker::current_state() const
{
    std::lock_guard<std::mutex> state_lock(current_state_mutex_);
    return current_state_;
}

void FusionTracker::joints_obsrv_callback(
    const FusionTracker::JointsObsrv& joints_obsrv)
{
    std::lock_guard<std::mutex> lock(joints_obsrv_buffer_mutex_);

    JointsObsrvEntry entry;
    entry.timestamp = ros::Time::now().toSec();
    entry.obsrv = joints_obsrv;
    joints_obsrvs_buffer_.push_back(entry);

    if (joints_obsrvs_buffer_.size() > 10000) joints_obsrvs_buffer_.pop_front();
}

void FusionTracker::image_obsrv_callback(
    const sensor_msgs::Image& ros_image)
{
    std::lock_guard<std::mutex> lock(image_obsrvs_mutex_);
    //    image_obsrv_ = image_obsrv;
    ros_image_ = ros_image;
}

auto FusionTracker::update_with_joints(
    const FusionTracker::JointsObsrv& joints_obsrv) -> State
{
    return gaussian_joint_tracker_->track(joints_obsrv);
}
}
