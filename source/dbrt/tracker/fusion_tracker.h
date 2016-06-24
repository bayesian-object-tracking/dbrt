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
 * \file fusion_tracker.hpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <list>
#include <deque>
#include <vector>
#include <mutex>
#include <memory>
#include <thread>
#include <functional>

#include <fl/filter/gaussian/gaussian_filter_linear.hpp>
#include <fl/model/observation/linear_gaussian_observation_model.hpp>
#include <fl/model/process/linear_state_transition_model.hpp>
#include <fl/model/process/interface/state_transition_function.hpp>
#include <dbrt/tracker/robot_tracker.h>
#include <dbrt/tracker/rotary_tracker.h>
#include <dbrt/tracker/visual_tracker.h>

namespace dbrt
{
/**
 * \brief VisualTracker
 */
class FusionTracker
{
public:
    // entire state and joint observation space
    typedef RotaryTracker::State State;
    typedef RotaryTracker::Obsrv JointsObsrv;
    typedef RotaryTracker::JointBelief JointBelief;

    // single joint observation space
    typedef RotaryTracker::JointObsrv JointObsrv;

    typedef std::function<std::shared_ptr<VisualTracker>()>
        VisualTrackerFactory;

    struct JointsObsrvEntry
    {
        double timestamp;
        JointsObsrv obsrv;
    };

    struct JointsBeliefEntry
    {
        JointsObsrvEntry joints_obsrv_entry;
        std::vector<JointBelief> beliefs;
    };

public:
    FusionTracker(const std::shared_ptr<dbot::CameraData>& camera_data,
                  const std::shared_ptr<RotaryTracker>& gaussian_joint_tracker,
                  const VisualTrackerFactory& visual_tracker_factory,
                  double camera_delay);

    /**
     * \brief Initializes the filters with the given initial states and
     *    the number of evaluations
     */
    void initialize(const std::vector<State>& initial_states);

    void run();
    void shutdown();

    void joints_obsrv_callback(const sensor_msgs::JointState& joints_obsrv);
    void image_obsrv_callback(const sensor_msgs::Image& ros_image);

    void current_state_and_time(State& current_state, double& current_time) const;
    void current_things(State& current_state, double& current_time,
        JointsObsrv& current_angle_measurement) const;

protected:
    void run_rotary_tracker();
    void run_visual_tracker();

private:
    int find_belief_entry(const std::deque<JointsBeliefEntry>& queue,
                          double timestamp,
                          JointsBeliefEntry& belief_entry);
    State get_state_from_belief(const JointsBeliefEntry& entry);
    Eigen::MatrixXd get_covariance_sqrt_from_belief(
        const JointsBeliefEntry& entry);

    std::vector<RotaryTracker::AngleBelief> get_angel_beliefs_from_moments(
        const State& mean,
        const Eigen::MatrixXd& cov);

private:
    VisualTrackerFactory visual_tracker_factory_;
    std::shared_ptr<dbot::CameraData> camera_data_;
    std::shared_ptr<RotaryTracker> gaussian_joint_tracker_;

    bool running_;
    double camera_delay_;

    State current_state_;
    // We need this to publish estimated tfs with the stamp corresponding to the current state.
    double current_time_;
    // We need this to calculate "measured" tfs at the same point in time.
    JointsObsrv current_angle_measurement_;

    sensor_msgs::Image ros_image_;
    bool ros_image_updated_;
    std::deque<JointsObsrvEntry> joints_obsrvs_buffer_;
    std::deque<JointsBeliefEntry> joints_obsrv_belief_buffer_;

    mutable std::mutex joints_obsrv_buffer_mutex_;
    mutable std::mutex joints_obsrv_belief_buffer_mutex_;
    mutable std::mutex image_obsrvs_mutex_;
    mutable std::mutex current_state_mutex_;
    std::thread gaussian_tracker_thread_;
    std::thread particle_tracker_thread_;
};
}
