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
 * \file rbc_particle_filter_tracker_builder.hpp
 * \date November 2015
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <exception>

#include <dbot/util/object_resource_identifier.hpp>
#include <dbot/tracker/builder/rb_observation_model_cpu_builder.hpp>

#include <brt/trackers/rbc_particle_filter_robot_tracker.hpp>
#include <brt/trackers/builder/robot_joint_transition_model_builder.hpp>

namespace brt
{
/**
 * \brief Represents an Rbc Particle filter based tracker builder
 */
class RbcParticleFilterRobotTrackerBuilder
{
public:
    typedef RobotState<> State;
    typedef Eigen::VectorXd Noise;
    typedef Eigen::VectorXd Input;

    typedef fl::StateTransitionFunction<State, Noise, Input> StateTransition;
    typedef dbot::RbObservationModel<State> ObservationModel;
    typedef typename ObservationModel::Observation Obsrv;

    typedef dbot::RaoBlackwellCoordinateParticleFilter<StateTransition,
                                                       ObservationModel> Filter;

    struct Parameters
    {
        struct TrackerParmeters
        {
            int evaluation_count;
            int max_sample_count;
            double update_rate;
            double max_kl_divergence;
        };

        bool use_gpu;

        TrackerParmeters cpu;
        TrackerParmeters gpu;
        TrackerParmeters tracker;

        dbot::RbObservationModelBuilder<State>::Parameters observation;
        RobotJointTransitionModelBuilder<State, Input>::Parameters
            state_transition;
    };

    enum SamplingBlockMethod
    {
        JointWise,
        ArmWise,
        RobotWise
    };

public:
    /**
     * \brief Creates a RbcParticleFilterTrackerBuilder
     * \param param			Builder and sub-builder parameters
     * \param camera_data	Tracker camera data object
     */
    RbcParticleFilterRobotTrackerBuilder(
        const Parameters& param,
        const dbot::CameraData& camera_data,
        const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics);

    /**
     * \brief Builds the Rbc PF tracker
     */
    std::shared_ptr<RbcParticleFilterRobotTracker> build();

    /**
     * \brief Creates an instance of the Rbc particle filter
     */
    std::shared_ptr<Filter> create_filter(const dbot::ObjectModel& object_model,
                                          double max_kl_divergence);

    /**
     * \brief Creates a linear state transition function used in the
     *        filter
     */
    std::shared_ptr<StateTransition> create_state_transition_model(
        const brt::RobotJointTransitionModelBuilder<State, Input>::Parameters&
            param) const;

    /**
     * \brief Creates the Rbc particle filter observation model. This can either
     *        be CPU or GPU based
     *
     * \throws NoGpuSupportException if compile with DBOT_BUILD_GPU=OFF and
     *         attempting to build a tracker with GPU support
     */
    std::shared_ptr<ObservationModel> create_obsrv_model(
        bool use_gpu,
        const dbot::ObjectModel& object_model,
        const dbot::CameraData& camera_data,
        const dbot::RbObservationModelBuilder<State>::Parameters& param) const;

    /**
     * \brief Loads and creates an object model represented by the specified
     *        resource identifier
     */
    dbot::ObjectModel create_object_model() const;

    /**
     * \brief Creates a sampling block definition used by the coordinate
     *        particle filter
     *
     * \param blocks		Number of objects or object parts
     * \param block_size	State dimension of each part
     */
    std::vector<std::vector<size_t>> create_sampling_blocks(int blocks) const;

private:
    Parameters param_;
    dbot::CameraData camera_data_;
    std::shared_ptr<KinematicsFromURDF> urdf_kinematics_;
};
}
