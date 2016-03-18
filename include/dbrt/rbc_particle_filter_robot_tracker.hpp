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
 * \file rbc_particle_filter_robot_tracker.hpp
 * \date December 2015
 * \author Jan Issac (jan.issac@gmail.com)
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#pragma once

#include <image_transport/image_transport.h>

#include <fl/model/process/interface/state_transition_function.hpp>

#include <dbot/rao_blackwell_coordinate_particle_filter.hpp>

#include <dbrt/robot_tracker.hpp>

namespace dbrt
{
/**
 * \brief RbcParticleFilterRobotTracker
 */
class RbcParticleFilterRobotTracker : public RobotTracker
{
public:
    typedef fl::StateTransitionFunction<State, Noise, Input> StateTransition;
    typedef dbot::RbObservationModel<State> ObservationModel;

    typedef dbot::RaoBlackwellCoordinateParticleFilter<StateTransition,
                                                       ObservationModel> Filter;

public:
    /**
     * \brief Creates the tracker
     *
     * \param filter
     *     Rbc particle filter instance
     * \param object_model
     *     Object model instance
     * \param camera_data
     *     Camera data container
     * \param update_rate
     *     Moving average update rate
     */
    RbcParticleFilterRobotTracker(const std::shared_ptr<Filter>& filter,
        const std::shared_ptr<dbot::ObjectModel>& object_model,
        const std::shared_ptr<dbot::CameraData>& camera_data,
        int evaluation_count,
        int block_count);

    /**
     * \brief perform a single filter step
     *
     * \param image
     *     Current observation image
     */
    State on_track(const Obsrv& image);

    /**
     * \brief Initializes the particle filter with the given initial states and
     *    the number of evaluations
     * @param initial_states
     * @param evaluation_count
     */
    State on_initialize(const std::vector<State>& initial_states,
                        const Eigen::VectorXd& obsrv);

    /**
     * \brief Returns camera data
     */
    const std::shared_ptr<dbot::CameraData>& camera_data() const;

private:
    std::shared_ptr<dbot::ObjectModel> object_model_;
    std::shared_ptr<dbot::CameraData> camera_data_;
    std::shared_ptr<Filter> filter_;
    int evaluation_count_;
    int block_count_;
};
}
