/*
 * This is part of the Bayesian Robot Tracking
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
 * \file fusion_robot_tracker.cpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <ros/ros.h>
#include <dbrt/fusion_robot_tracker.h>

namespace dbrt
{

FusionRobotTracker::FusionRobotTracker(
    const std::shared_ptr<GaussianJointFilterRobotTracker>&
        gaussian_joint_tracker,
    const std::shared_ptr<RbcParticleFilterRobotTracker>&
        rbc_particle_filter_tracker)
    : gaussian_joint_tracker_(gaussian_joint_tracker),
      rbc_particle_filter_tracker_(rbc_particle_filter_tracker),
      running_(true)
{
}

void FusionRobotTracker::initialize(const std::vector<State>& initial_states,
                                    const Eigen::VectorXd& obsrv)
{
    current_state_ = initial_states[0];
    gaussian_joint_tracker_->initialize(initial_states, obsrv);
    rbc_particle_filter_tracker_->ini
}

void FusionRobotTracker::run_gaussian_tracker()
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
            current_state_ = current_state;
        }
    }
}

void FusionRobotTracker::run_particle_tracker()
{
    while (running_)
    {
        usleep(1);

    }
}

void FusionRobotTracker::run()
{
    running_ = true;
    gaussian_tracker_thread_ =
        std::thread(&FusionRobotTracker::run_gaussian_tracker, this);
}

void FusionRobotTracker::shutdown()
{
    running_ = false;
    gaussian_tracker_thread_.join();
}

FusionRobotTracker::State FusionRobotTracker::current_state() const
{
    std::lock_guard<std::mutex> state_lock(current_state_mutex_);
    return current_state_;
}

void FusionRobotTracker::joints_obsrv_callback(
    const FusionRobotTracker::JointsObsrv& joints_obsrv)
{
    std::lock_guard<std::mutex> lock(joints_obsrv_buffer_mutex_);

    JointsObsrvEntry entry;
    entry.timestamp = ros::Time::now().toSec();
    entry.obsrv = joints_obsrv;
    joints_obsrvs_buffer_.push_back(entry);

    if (joints_obsrvs_buffer_.size() > 10000) joints_obsrvs_buffer_.pop_front();
}

void FusionRobotTracker::image_obsrv_callback(
    const Eigen::VectorXd& image_obsrv)
{
    std::lock_guard<std::mutex> lock(image_obsrvs_mutex_);
    image_obsrv_ = image_obsrv;
}

auto FusionRobotTracker::update_with_joints(
    const FusionRobotTracker::JointsObsrv& joints_obsrv) -> State
{
    return gaussian_joint_tracker_->track(joints_obsrv);
}
}
