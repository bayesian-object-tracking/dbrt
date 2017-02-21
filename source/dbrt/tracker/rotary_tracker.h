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
 * \file gaussian_joint_robot_tracker.h
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <dbrt/kinematics_from_urdf.h>
#include <dbrt/tracker/robot_tracker.h>
#include <fl/filter/gaussian/gaussian_filter_linear.hpp>
#include <fl/model/sensor/linear_gaussian_sensor.hpp>
#include <fl/model/transition/interface/transition_function.hpp>
#include <fl/model/transition/linear_transition.hpp>
#include <memory>
#include <mutex>
#include <sensor_msgs/JointState.h>

namespace dbrt
{
/**
 * \brief VisualTracker
 */
class RotaryTracker : public RobotTracker
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
    typedef fl::LinearTransition<JointState, JointNoise, JointInput>
        JointStateModel;

    // Linear observation function
    typedef fl::LinearGaussianSensor<JointObsrv, JointState> JointSensor;

    // Kalman filter for a single joint
    typedef fl::GaussianFilter<JointStateModel, JointSensor> JointFilter;

    // Belief representation of a single joint, i.e. a Gaussian
    typedef typename JointFilter::Belief JointBelief;

    // Belief distribution of a single joint
    typedef fl::Gaussian<Eigen::Matrix<fl::Real, 1, 1>> AngleBelief;

public:
    RotaryTracker(
        const std::shared_ptr<std::vector<JointFilter>>& joint_filters,
        const std::shared_ptr<KinematicsFromURDF>& kinematics);

    /**
     * \brief perform a single filter step
     *
     * \param image
     *     Current observation image
     */
    State track(const Obsrv& joints_obsrv);

    /**
     * \brief Initializes the particle filter with the given initial states and
     *    the number of evaluations
     * \param initial_states
     *    initial set of states (in most cases a single state)
     * \param obsrv
     *    initial observation which may be required in
     */
    void initialize(const std::vector<State>& initial_states);

    /**
    void track_callback(const sensor_msgs::JointState &joint_msg);

    /**
     * \brief DOC please!
     */
    std::vector<AngleBelief> angle_beliefs();

    /**
     * \brief DOC please!
     */
    void set_angle_beliefs(std::vector<AngleBelief> angle_beliefs);

    void set_beliefs(const std::vector<JointBelief>& beliefs);

    /**
     * \brief Returns immutable reference to all joint belliefs
     */
    const std::vector<JointBelief>& beliefs() const;

    /**
     * \brief Returns mutable reference to all joint belliefs
     */
    std::vector<JointBelief>& beliefs();

    /**
     * \brief Returns current state from the belief
     */
    State current_state() const;

    /**
     * Callback function to apply the filter for the given joint state message
     */
    void track_callback(const sensor_msgs::JointState& joint_msg);

private:
    /* std::vector<int> joint_order_; */
    std::shared_ptr<KinematicsFromURDF> kinematics_;
    State current_state_;
    std::vector<JointBelief> beliefs_;
    std::shared_ptr<std::vector<JointFilter>> joint_filters_;
};
}
