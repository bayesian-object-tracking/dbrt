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
 * \file particle_tracker_builder.h
 * \date November 2015
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <dbot/builder/particle_tracker_builder.h>
#include <dbot/object_resource_identifier.h>
#include <dbot/tracker/tracker.h>
#include <dbrt/builder/exceptions.h>
#include <dbrt/builder/transition_builder.h>
#include <dbrt/kinematics_from_urdf.h>
#include <dbrt/tracker/visual_tracker.h>
#include <exception>

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
    typedef dbrt::TransitionBuilder<Tracker> TransitionBuilder;
    typedef dbot::RbSensorBuilder<State> SensorBuilder;

    /* == Model Interfaces ================================================== */
    typedef fl::TransitionFunction<State, Noise, Input> Transition;
    typedef dbot::RbSensor<State> Sensor;
    typedef typename Sensor::Observation Obsrv;

    /* == Filter algorithm ================================================== */
    typedef dbot::RaoBlackwellCoordinateParticleFilter<Transition, Sensor>
        Filter;

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
        const std::shared_ptr<TransitionBuilder>& transition_builder,
        const std::shared_ptr<SensorBuilder>& sensor_builder,
        const std::shared_ptr<dbot::ObjectModel>& object_model,
        const std::shared_ptr<dbot::CameraData>& camera_data,
        const Parameters& params)
        : transition_builder_(transition_builder),
          sensor_builder_(sensor_builder),
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

        auto transition = this->transition_builder_->build();
        auto sensor = this->sensor_builder_->build();

        auto filter = std::make_shared<Filter>(
            transition, sensor, params_.sampling_blocks, max_kl_divergence);
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
    std::shared_ptr<TransitionBuilder> transition_builder_;
    std::shared_ptr<SensorBuilder> sensor_builder_;
    std::shared_ptr<dbot::ObjectModel> object_model_;
    std::shared_ptr<dbot::CameraData> camera_data_;
    Parameters params_;
    std::shared_ptr<KinematicsFromURDF> urdf_kinematics_;
};
}
