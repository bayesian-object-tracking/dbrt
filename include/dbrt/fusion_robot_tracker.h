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

namespace dbrt
{
/**
 * \brief RbcParticleFilterRobotTracker
 */
class FusionRobotTracker : public RobotTracker
{
public:
    // single joint filter
    typedef Eigen::Matrix<fl::Real, 1, 1> JointState;
    typedef Eigen::Matrix<fl::Real, 1, 1> JointNoise;
    typedef Eigen::Matrix<fl::Real, 1, 1> JointObsrv;
    typedef Eigen::Matrix<fl::Real, 1, 1> JointInput;
    typedef fl::LinearStateTransitionModel<JointState, JointNoise, JointInput>
        JointStateModel;
    typedef fl::LinearGaussianObservationModel<JointObsrv, JointState>
        JointObsrvModel;
    typedef fl::GaussianFilter<JointStateModel, JointObsrvModel> JointFilter;

    // augmented joint observation
    typedef Eigen::Matrix<fl::Real, Eigen::Dynamic, 1> JointsObsrv;

    typedef typename JointFilter::Belief Belief;

public:
    FusionRobotTracker(
        const std::shared_ptr<std::vector<JointFilter>>& joint_filter,
        const std::shared_ptr<dbot::ObjectModel>& object_model,
        const std::shared_ptr<dbot::CameraData>& camera_data);

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

    State on_track(const Obsrv& image) { }

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
    std::vector<Belief> beliefs_;
    std::shared_ptr<std::vector<JointFilter>> joint_filters_;
};
}
