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
 * \file fusion_robot_tracker.hpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <mutex>
#include <memory>

#include <fl/filter/gaussian/gaussian_filter_linear.hpp>
#include <fl/model/observation/linear_gaussian_observation_model.hpp>
#include <fl/model/process/linear_state_transition_model.hpp>
#include <fl/model/process/interface/state_transition_function.hpp>
#include <dbrt/robot_tracker.hpp>
#include <dbrt/gaussian_joint_filter_robot_tracker.hpp>

namespace dbrt
{
/**
 * \brief RbcParticleFilterRobotTracker
 */
class FusionRobotTracker
{
public:
    typedef GaussianJointFilterRobotTracker::State State;
    typedef GaussianJointFilterRobotTracker::Obsrv JointsObsrv;
    typedef GaussianJointFilterRobotTracker::Belief GaussianJointTrackerBelief;

public:
    FusionRobotTracker(const std::shared_ptr<GaussianJointFilterRobotTracker>&
                           gaussian_joint_tracker);



    /**
     * \brief perform a single filter step
     *
     * \param image
     *     Current observation image
     */
    State track(const Obsrv& image, const JointsObsrv& joints_obsrv)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto state = on_track(image, joints_obsrv);
        return state;
    }

    State on_track(const Obsrv& image) {}
    /**
     * \brief perform a single filter step
     *
     * \param image
     *     Current observation image
     */
    State on_track(const Obsrv& image, const JointsObsrv& joints_obsrv);

    /**
     * \brief Initializes the particle filter with the given initial states and
     *    the number of evaluations
     * @param initial_states
     * @param evaluation_count
     */
    State on_initialize(const std::vector<State>& initial_states,
                        const Eigen::VectorXd& obsrv);

private:
    std::vector<GaussianJointTrackerBelief> gaussian_joint_tracker_beliefs_;
    std::shared_ptr<GaussianJointFilterRobotTracker> gaussian_joint_tracker_;
};
}
