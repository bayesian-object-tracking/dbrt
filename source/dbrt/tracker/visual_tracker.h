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
 * \file visual_tracker.hpp
 * \date December 2015
 * \author Jan Issac (jan.issac@gmail.com)
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#pragma once

#include <image_transport/image_transport.h>

#include <fl/model/transition/interface/transition_function.hpp>

#include <dbot/filter/rao_blackwell_coordinate_particle_filter.hpp>

#include <dbrt/tracker/robot_tracker.h>

namespace dbrt
{
/**
 * \brief VisualTracker
 */
class VisualTracker : public RobotTracker
{
public:
    typedef fl::TransitionFunction<State, Noise, Input> Transition;
    typedef dbot::RbSensor<State> Sensor;

    typedef dbot::RaoBlackwellCoordinateParticleFilter<Transition,
                                                       Sensor> Filter;

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
    VisualTracker(const std::shared_ptr<Filter>& filter,
        const std::shared_ptr<dbot::ObjectModel>& object_model,
        int evaluation_count,
        int block_count);

    /**
     * \brief perform a single filter step
     *
     * \param image
     *     Current observation image
     */
    State track(const Obsrv& image);

    /**
     * \brief Initializes the particle filter with the given initial states and
     *    the number of evaluations
     * @param initial_states
     * @param evaluation_count
     */
    void initialize(const std::vector<State>& initial_states);

    const std::shared_ptr<Filter> filter();

private:
    std::shared_ptr<dbot::ObjectModel> object_model_;
    std::shared_ptr<Filter> filter_;
    int evaluation_count_;
    int block_count_;
};
}
