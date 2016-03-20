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

/*
 * This file implements a part of the algorithm published in:
 *
 * M. Wuthrich, J. Bohg, D. Kappler, C. Pfreundt, S. Schaal
 * The Coordinate Particle Filter -
 * A novel Particle Filter for High Dimensional Systems
 * IEEE Intl Conf on Robotics and Automation, 2015
 * http://arxiv.org/abs/1505.00251
 *
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

#include <dbrt/visual_tracker.hpp>
#include <dbrt/util/kinematics_from_urdf.hpp>
#include <dbrt/util/builder/transition_builder.hpp>
#include <dbrt/util/builder/exceptions.hpp>

namespace dbrt
{

template <typename Tracker>
class VisualTrackerBuilder
{
public:
    typedef typename Tracker::State State;
    typedef typename Tracker::Noise Noise;
    typedef typename Tracker::Input Input;

    /* == Model Builder Interfaces ========================================== */
    typedef dbrt::TransitionBuilder<Tracker>
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
    VisualTrackerBuilder(
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
    std::shared_ptr<VisualTracker> build()
    {
        auto filter =
            create_filter(this->object_model_, this->params_.max_kl_divergence);

        auto tracker = std::make_shared<VisualTracker>(
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
