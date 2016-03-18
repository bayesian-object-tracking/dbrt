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
 * \file gaussian_joint_robot_tracker.hpp
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
class GaussianJointFilterRobotTracker : public RobotTracker
{
public:
    enum Dimension
    {
        JointStateDim = 2,
        JointNoiseDim = 2,
        JointObsrvDim = 1,
        JointInputDim = 1,
    };

    // single joint filter
    typedef Eigen::Matrix<fl::Real, JointStateDim, 1> JointState;
    typedef Eigen::Matrix<fl::Real, JointNoiseDim, 1> JointNoise;
    typedef Eigen::Matrix<fl::Real, JointObsrvDim, 1> JointObsrv;
    typedef Eigen::Matrix<fl::Real, JointInputDim, 1> JointInput;

    // Linear state transition function
    typedef fl::LinearStateTransitionModel<JointState, JointNoise, JointInput>
        JointStateModel;

    // Linear observation function
    typedef fl::LinearGaussianObservationModel<JointObsrv, JointState>
        JointObsrvModel;

    // Kalman filter for a single joint
    typedef fl::GaussianFilter<JointStateModel, JointObsrvModel> JointFilter;

    // Belief representation of a single joint, i.e. a Gaussian
    typedef typename JointFilter::Belief JointBelief;


    typedef fl::Gaussian<Eigen::Matrix<fl::Real, 1, 1>> AngleBelief;


public:
    GaussianJointFilterRobotTracker(
        const std::shared_ptr<std::vector<JointFilter>>& joint_filters);

    /**
     * \brief perform a single filter step
     *
     * \param image
     *     Current observation image
     */
    State on_track(const Obsrv& joints_obsrv);

    /**
     * \brief Initializes the particle filter with the given initial states and
     *    the number of evaluations
     * \param initial_states
     *    initial set of states (in most cases a single state)
     * \param obsrv
     *    initial observation which may be required in
     */
    State on_initialize(const std::vector<State>& initial_states,
                        const Eigen::VectorXd& obsrv);


    std::vector<AngleBelief> angle_beliefs();

    void set_angle_beliefs(std::vector<AngleBelief> angle_beliefs);



    /**
     * \brief Returns immutable reference to all joint belliefs
     */
    const std::vector<JointBelief>& beliefs() const;

    /**
     * \brief Returns mutable reference to all joint belliefs
     */
    std::vector<JointBelief>& beliefs();

private:
    std::vector<JointBelief> beliefs_;
    std::shared_ptr<std::vector<JointFilter>> joint_filters_;
};
}
