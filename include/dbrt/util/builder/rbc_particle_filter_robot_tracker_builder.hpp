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
#include <dbot/tracker/object_tracker.hpp>
#include <dbot/tracker/builder/rbc_particle_filter_tracker_builder.hpp>

#include <dbrt/rbc_particle_filter_robot_tracker.hpp>
#include <dbrt/util/kinematics_from_urdf.hpp>
#include <dbrt/util/builder/robot_joint_transition_model_builder.hpp>

namespace dbrt
{
/**
 * \brief Represents an exception thrown if the number of indices in the
 * sampling block does not match the state dimension.
 */
class InvalidNumberOfSamplingBlocksException : public std::exception
{
public:
    const char* what() const noexcept
    {
        return "The number of indices in the sampling blocks does not match the "
               "number of joints (joint state dimension) of the robot.";
    }
};

template <typename Tracker>
class RbcParticleFilterRobotTrackerBuilder
{
public:
    typedef typename Tracker::State State;
    typedef typename Tracker::Noise Noise;
    typedef typename Tracker::Input Input;

    /* == Model Builder Interfaces ========================================== */
    typedef dbot::StateTransitionFunctionBuilder<State, Noise, Input>
        StateTransitionBuilder;
    typedef dbot::RbObservationModelBuilder<State> ObservationModelBuilder;

    /* == Model Interfaces ================================================== */
    typedef fl::StateTransitionFunction<State, Noise, Input> StateTransition;
    typedef dbot::RbObservationModel<State> ObservationModel;
    typedef typename ObservationModel::Observation Obsrv;

    /* == Filter algorithm ================================================== */
    typedef dbot::RaoBlackwellCoordinateParticleFilter<StateTransition,
                                                       ObservationModel> Filter;

    /* == Tracker parameters ================================================ */
    struct Parameters
    {
        int evaluation_count;
        double moving_average_update_rate;
        double max_kl_divergence;
        std::vector<std::vector<int>> sampling_blocks;
    };

public:
    RbcParticleFilterRobotTrackerBuilder(
        const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics,
        const std::shared_ptr<StateTransitionBuilder>& state_transition_builder,
        const std::shared_ptr<ObservationModelBuilder>& obsrv_model_builder,
        const std::shared_ptr<dbot::ObjectModel>& object_model,
        const std::shared_ptr<dbot::CameraData>& camera_data,
        const Parameters& params)
        : state_transition_builder_(state_transition_builder),
          obsrv_model_builder_(obsrv_model_builder),
          object_model_(object_model),
          camera_data_(camera_data),
          params_(params),
          urdf_kinematics_(urdf_kinematics)
    {
    }

    /**
     * \brief Builds the Rbc PF tracker
     */
    std::shared_ptr<RbcParticleFilterRobotTracker> build()
    {
        auto filter =
            create_filter(this->object_model_, this->params_.max_kl_divergence);

        auto tracker = std::make_shared<RbcParticleFilterRobotTracker>(
            filter,
            this->object_model_,
            this->camera_data_,
            this->params_.evaluation_count,
            this->params_.sampling_blocks.size());

        return tracker;
    }

    virtual std::shared_ptr<Filter> create_filter(
        const std::shared_ptr<dbot::ObjectModel>& object_model,
        double max_kl_divergence)
    {
        if (count_sampling_block_indices(params_.sampling_blocks) !=
            urdf_kinematics_->num_joints())
        {
            throw InvalidNumberOfSamplingBlocksException();
        }

        auto state_transition_model = this->state_transition_builder_->build();
        auto obsrv_model = this->obsrv_model_builder_->build();

        //        auto sampling_blocks =
        //            this->create_sampling_blocks(urdf_kinematics_->num_joints(),
        //            1);

        auto filter = std::make_shared<Filter>(state_transition_model,
                                               obsrv_model,
                                               params_.sampling_blocks,
                                               max_kl_divergence);
        return filter;
    }
    /**
     * \brief Creates a sampling block definition used by the coordinate
     *        particle filter
     *
     * \param blocks		Number of objects or object parts
     * \param block_size	State dimension of each part
     */
    virtual std::vector<std::vector<int>> create_sampling_blocks(
        int blocks,
        int block_size) const
    {
        std::vector<std::vector<int>> sampling_blocks(blocks);
        for (int i = 0; i < blocks; ++i)
        {
            for (int k = 0; k < block_size; ++k)
            {
                sampling_blocks[i].push_back(i * block_size + k);
            }
        }

        return sampling_blocks;
    }

    /**
     * \brief Counts the number of indices in the sampling blocks
     */
    int count_sampling_block_indices(
        const std::vector<std::vector<int>>& sampling_blocks) const
    {
        int index_count = 0;

        for (auto& block : sampling_blocks) index_count += block.size();

        return index_count;
    }

protected:
    std::shared_ptr<StateTransitionBuilder> state_transition_builder_;
    std::shared_ptr<ObservationModelBuilder> obsrv_model_builder_;
    std::shared_ptr<dbot::ObjectModel> object_model_;
    std::shared_ptr<dbot::CameraData> camera_data_;
    Parameters params_;
    std::shared_ptr<KinematicsFromURDF> urdf_kinematics_;
};
}
