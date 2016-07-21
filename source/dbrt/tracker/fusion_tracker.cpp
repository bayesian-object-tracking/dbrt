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
#include <sensor_msgs/JointState.h>
#include <dbrt/tracker/fusion_tracker.h>

#include <dbot_ros/util/ros_interface.hpp>

namespace dbrt
{
FusionTracker::FusionTracker(
    const std::shared_ptr<dbot::CameraData>& camera_data,
    const std::shared_ptr<RotaryTracker>& gaussian_joint_tracker,
    const VisualTrackerFactory& visual_tracker_factory,
    double camera_delay)
    : camera_data_(camera_data),
      gaussian_joint_tracker_(gaussian_joint_tracker),
      visual_tracker_factory_(visual_tracker_factory),
      running_(true),
      camera_delay_(camera_delay),
      ros_image_updated_(false)
{
}

void FusionTracker::initialize(const std::vector<State>& initial_states)
{
    current_state_ = initial_states[0];
    gaussian_joint_tracker_->initialize(initial_states);
}

void FusionTracker::run_rotary_tracker()
{
    while (running_)
    {
        usleep(10);
        std::deque<JointsObsrvEntry> joints_obsrvs_buffer_local;
        {
            std::lock_guard<std::mutex> lock(joints_obsrv_buffer_mutex_);
            if (joints_obsrvs_buffer_.size() == 0) continue;
            joints_obsrvs_buffer_.swap(joints_obsrvs_buffer_local);
        }

        State current_state;
        double current_time;
        JointsObsrv current_angle_measurement;
        {
            std::lock_guard<std::mutex> state_lock(current_state_mutex_);
            current_state = current_state_;
            current_time = current_time_;
            current_angle_measurement = current_angle_measurement_;
        }


        std::lock_guard<std::mutex> belief_buffer_lock(
            joints_obsrv_belief_buffer_mutex_);
        for (auto joints_obsrv_entry : joints_obsrvs_buffer_local)
        {
            // construct a joints belief entry which contains the following
            //  - the joints measurement values
            //  - their time stamp
            //  - the updated joints belief
            JointsBeliefEntry joints_belief_entry;
            joints_belief_entry.joints_obsrv_entry = joints_obsrv_entry;

            current_state = gaussian_joint_tracker_->track(
                joints_belief_entry.joints_obsrv_entry.obsrv);
            current_time = joints_belief_entry.joints_obsrv_entry.timestamp;
            current_angle_measurement =
                                   joints_belief_entry.joints_obsrv_entry.obsrv;

            joints_belief_entry.beliefs =
                gaussian_joint_tracker_->beliefs();

            // update sliding window of belief and joints obsrv entries
            joints_obsrv_belief_buffer_.push_back(joints_belief_entry);
            if (joints_obsrv_belief_buffer_.size() > 10000)
            {
                PInfo("Belief buffer max size reached ... popping");
                joints_obsrv_belief_buffer_.pop_front();
            }
        }

        {
            std::lock_guard<std::mutex> state_lock(current_state_mutex_);
            current_state_ = current_state;
            current_time_ = current_time;
            current_angle_measurement_ = current_angle_measurement;
        }
    }
}

void FusionTracker::run_visual_tracker()
{

    std::shared_ptr<VisualTracker> rbc_particle_filter_tracker =
        visual_tracker_factory_();

    State current_state;
    double garbage;

    current_state_and_time(current_state, garbage);
    rbc_particle_filter_tracker->initialize({current_state});

    while (running_)
    {
        // continue only if there is a new image available
        usleep(10);
        {
            std::lock_guard<std::mutex> lock(image_obsrvs_mutex_);
            if (!ros_image_updated_)
            {
                continue;
            }
        }

        /**
         * #1 SWAP ROTARY BELIEF QUEUES SECURLY
         * #2 GET ROTARY BELIEF AND ITS INDEX FOR IMAGE TIMESTAMP
         * #3 CONSTRUCT STATE AND NOISE MATRIX FROM ROTARY BELIEF
         * #4 GET PROCESS MODEL
         * #5 SET PROCESS MODEL NOISE COVARIANCE
         * #6 INITIALIZE PARTICLE FILTER WITH ROTARY STATE
         * #7 TRACK AND GET STATE AND COVARIANCE
         * #8 CONSTRUCT NEW ANGEL BELIEFS
         * #9 SET ROTARY ANGEL BELIEFS
         * #10 FILL BUFFER WITH OLD JOINT OBSRV
         */

        INIT_PROFILING;

        JointsBeliefEntry belief_entry;
        int belief_index;

        // #1
        std::deque<JointsBeliefEntry> joints_obsrv_belief_buffer_local;
        {
            std::lock_guard<std::mutex> belief_buffer_lock(
                joints_obsrv_belief_buffer_mutex_);

            joints_obsrv_belief_buffer_.swap(joints_obsrv_belief_buffer_local);
        }

        // #2
        belief_index = find_belief_entry(joints_obsrv_belief_buffer_local,
                                         ros_image_.header.stamp.toSec(),
                                         belief_entry);
        if (belief_index < 0)
        {
            std::cout << "belief_index < 0: " << belief_index << std::endl;

            std::lock_guard<std::mutex> belief_buffer_lock(
                joints_obsrv_belief_buffer_mutex_);
            while (joints_obsrv_belief_buffer_local.size() > 0)
            {
                joints_obsrv_belief_buffer_.push_front(
                    joints_obsrv_belief_buffer_local.back());
                joints_obsrv_belief_buffer_local.pop_back();
            }
            continue;
        }


        // // just some print -----------------------------------------------------
        // static ros::Time last_image_stamp;
        // ros::Duration delta = ros_image_.header.stamp - last_image_stamp;
        // last_image_stamp = ros_image_.header.stamp;
        // std::cout << "time difference between processed images: " << delta <<
        //              std::endl;
        // // ---------------------------------------------------------------------

        // #3
        auto mean = get_state_from_belief(belief_entry);
        auto cov_sqrt = get_covariance_sqrt_from_belief(belief_entry);

        // #4
        auto transition = std::static_pointer_cast<
            fl::LinearTransition<VisualTracker::State,
                                           VisualTracker::Noise,
                                           VisualTracker::Input>>(
            rbc_particle_filter_tracker->filter()->transition());

        // #5
//        std::cout << "setting covariance sqrt " << std::endl
//                     << cov_sqrt << std::endl;


        transition->noise_matrix(cov_sqrt);

        // #6
        rbc_particle_filter_tracker->initialize({mean});

        // #7
        sensor_msgs::Image ros_image;
        {
            std::lock_guard<std::mutex> lock(image_obsrvs_mutex_);
            ros_image = ros_image_;
            ros_image_updated_ = false;
        }


        auto image = ri::to_eigen_vector<double>(
            ros_image, camera_data_->downsampling_factor());
        State current_state;
        current_state = rbc_particle_filter_tracker->track(image);
        auto cov = rbc_particle_filter_tracker->filter()->belief().covariance();

        // #8
        auto angle_beliefs = get_angel_beliefs_from_moments(current_state, cov);

        // #9
        std::lock_guard<std::mutex> belief_buffer_lock(
            joints_obsrv_belief_buffer_mutex_);
        std::lock_guard<std::mutex> lock(joints_obsrv_buffer_mutex_);

        gaussian_joint_tracker_->set_beliefs(belief_entry.beliefs);
        gaussian_joint_tracker_->set_angle_beliefs(angle_beliefs);

        while (joints_obsrv_belief_buffer_.size() > 0)
        {
            joints_obsrvs_buffer_.push_front(
                joints_obsrv_belief_buffer_.back().joints_obsrv_entry);
            joints_obsrv_belief_buffer_.pop_back();
        }

        // throw away obsrv prior to belief index
        for (int i = 0; i < belief_index; ++i)
        {
            joints_obsrv_belief_buffer_local.pop_front();
        }

        while (joints_obsrv_belief_buffer_local.size() > 0)
        {
            joints_obsrvs_buffer_.push_front(
                joints_obsrv_belief_buffer_local.back().joints_obsrv_entry);
            joints_obsrv_belief_buffer_local.pop_back();
        }

        // timestamp check
        double maxdelta = 0;
        double avdelta = 0;
        JointsObsrvEntry prev;
        prev.timestamp = 0;
        for (auto& entry: joints_obsrvs_buffer_)
        {
            if (entry.timestamp < prev.timestamp)
            {
                PV(entry.timestamp - prev.timestamp);
                PV(entry.obsrv);
                PV(prev.obsrv);
                PInfo("constructed queue is wrong");
            }

            if (prev.timestamp > 0)
            {
                double delta = entry.timestamp - prev.timestamp;
                maxdelta = std::max(maxdelta, delta);
                avdelta += delta;
            }

            prev = entry;
        }
        MEASURE("total time for visual processing");
    }

}

int FusionTracker::find_belief_entry(const std::deque<JointsBeliefEntry>& queue,
                                     double timestamp,
                                     JointsBeliefEntry& belief_entry)
{
    int index = 0;
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::min();
    for (const auto entry : queue)
    {
        min = std::min(min, entry.joints_obsrv_entry.timestamp);
        max = std::max(max, entry.joints_obsrv_entry.timestamp);

        if (entry.joints_obsrv_entry.timestamp > timestamp)
        {
            belief_entry = entry;
            return index;
        }
        index++;
    }

    std::cout.precision(20);

    std::cout << "could not find a matching belief for time stamp "
                 << timestamp << std::endl;
    std::cout << "latest belief: " << max << std::endl;
    std::cout << "oldest belief: " << min << std::endl;

    return -1;
}

auto FusionTracker::get_state_from_belief(const JointsBeliefEntry& entry)
    -> State
{
    State state;
    state.resize(entry.beliefs.size());
    for (int i = 0; i < state.size(); ++i)
    {
        state(i, 0) = entry.beliefs[i].mean()(0, 0);
    }

    return state;
}

Eigen::MatrixXd FusionTracker::get_covariance_sqrt_from_belief(
    const FusionTracker::JointsBeliefEntry& entry)
{
    Eigen::MatrixXd cov_sqrt;
    cov_sqrt.setZero(entry.beliefs.size(), entry.beliefs.size());

    if (entry.beliefs.size() == 0)
    {
        throw std::runtime_error("Something is wrong. The beliefs are empty.");
    }

    for (int i = 0; i < cov_sqrt.rows(); ++i)
    {
        cov_sqrt(i, i) = std::sqrt(entry.beliefs[i].covariance()(0, 0));
    }

    return cov_sqrt;
}

std::vector<RotaryTracker::AngleBelief>
FusionTracker::get_angel_beliefs_from_moments(const FusionTracker::State& mean,
                                              const Eigen::MatrixXd& cov)
{
    std::vector<RotaryTracker::AngleBelief> beliefs(mean.size());

    for (int i = 0; i < mean.size(); ++i)
    {
        RotaryTracker::AngleBelief belief;
        auto angle_mean = beliefs[i].mean();
        auto angle_cov = beliefs[i].covariance();
        angle_mean(0, 0) = mean(i, 0);
        angle_cov(0, 0) = cov(i, i);

        beliefs[i].mean(angle_mean);
        beliefs[i].covariance(angle_cov);
    }

    return beliefs;
}

void FusionTracker::run()
{
    running_ = true;
    gaussian_tracker_thread_ =
        std::thread(&FusionTracker::run_rotary_tracker, this);
    particle_tracker_thread_ =
        std::thread(&FusionTracker::run_visual_tracker, this);
}

void FusionTracker::shutdown()
{
    running_ = false;
    gaussian_tracker_thread_.join();
    particle_tracker_thread_.join();
}

void FusionTracker::current_state_and_time(State& current_state,
                                           double& current_time) const
{
    std::lock_guard<std::mutex> state_lock(current_state_mutex_);

    current_state = current_state_;
    current_time = current_time_;
}

void FusionTracker::current_things(State& current_state,
    double& current_time, JointsObsrv& current_angle_measurement) const
{
    std::lock_guard<std::mutex> state_lock(current_state_mutex_);

    current_state = current_state_;
    current_time = current_time_;
    current_angle_measurement = current_angle_measurement_;
}

void FusionTracker::joints_obsrv_callback(
    const sensor_msgs::JointState& joint_msg)
{
    std::lock_guard<std::mutex> lock(joints_obsrv_buffer_mutex_);

    /// hack: we add a measurement = 0 for the six extra joints corresponding
    /// to the camera offset ***************************************************
    sensor_msgs::JointState joint_state_with_offset = joint_msg;

    joint_state_with_offset.name.push_back("XTION_X");
    joint_state_with_offset.name.push_back("XTION_Y");
    joint_state_with_offset.name.push_back("XTION_Z");
    joint_state_with_offset.name.push_back("XTION_ROLL");
    joint_state_with_offset.name.push_back("XTION_PITCH");
    joint_state_with_offset.name.push_back("XTION_YAW");

    joint_state_with_offset.position.push_back(0);
    joint_state_with_offset.position.push_back(0);
    joint_state_with_offset.position.push_back(0);
    joint_state_with_offset.position.push_back(0);
    joint_state_with_offset.position.push_back(0);
    joint_state_with_offset.position.push_back(0);
    /// ************************************************************************

    const auto joint_order = gaussian_joint_tracker_->joint_order();

    Eigen::VectorXd obsrv(joint_state_with_offset.position.size());
    for (int i = 0; i < joint_state_with_offset.position.size(); ++i)
    {
        obsrv[joint_order[i]] = joint_state_with_offset.position[i];
    }

    JointsObsrvEntry entry;
    entry.timestamp = joint_state_with_offset.header.stamp.toSec();
    entry.obsrv = obsrv;
    joints_obsrvs_buffer_.push_back(entry);

    if (joints_obsrvs_buffer_.size() > 1000000)
    {
        ROS_WARN("Obsrv buffer max size reached.");
        joints_obsrvs_buffer_.pop_front();
    }
}

void FusionTracker::image_obsrv_callback(const sensor_msgs::Image& ros_image)
{
    std::lock_guard<std::mutex> lock(image_obsrvs_mutex_);

    ros_image_updated_ = true;
    ros_image_ = ros_image;
    ros_image_.header.stamp.fromSec(
                ros_image_.header.stamp.toSec() - camera_delay_);
}

}
