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

#include <fl/filter/gaussian/gaussian_filter_linear.hpp>
#include <fl/model/observation/linear_gaussian_observation_model.hpp>
#include <fl/model/process/linear_state_transition_model.hpp>
#include <fl/model/process/interface/state_transition_function.hpp>
#include <dbrt/robot_tracker.hpp>
#include <dbrt/rotary_tracker.hpp>
#include <dbrt/visual_tracker.hpp>

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

public:
    FusionTracker(const std::shared_ptr<RotaryTracker>&
                           gaussian_joint_tracker,
                       const std::shared_ptr<VisualTracker>&
                           rbc_particle_filter_tracker);

    /**
     * \brief Initializes the filters with the given initial states and
     *    the number of evaluations
     */
    void initialize(const std::vector<State>& initial_states);

    void run();
    void shutdown();

    /**
     * \brief perform a single filter step
     *
     * \param image
     *     Current observation image
     */
    State update_with_joints(const JointsObsrv& joints_obsrv);

    void joints_obsrv_callback(const JointsObsrv& joints_obsrv);
    void image_obsrv_callback(const Eigen::VectorXd& image_obsrv);

    State current_state() const;

protected:
    void run_gaussian_tracker();
    void run_particle_tracker();

protected:
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

private:
    bool running_;
    State current_state_;
    Eigen::VectorXd image_obsrv_;
    std::deque<JointsObsrvEntry> joints_obsrvs_buffer_;
    std::deque<JointsBeliefEntry> joints_obsrv_belief_buffer_;
    std::shared_ptr<RotaryTracker> gaussian_joint_tracker_;
    std::shared_ptr<VisualTracker> rbc_particle_filter_tracker_;

    mutable std::mutex joints_obsrv_buffer_mutex_;
    mutable std::mutex image_obsrvs_mutex_;
    mutable std::mutex current_state_mutex_;
    std::thread gaussian_tracker_thread_;
    std::thread particle_tracker__thread_;
};
}
