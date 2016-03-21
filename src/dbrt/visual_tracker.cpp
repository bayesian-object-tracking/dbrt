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
 * \file visual_tracker.cpp
 * \date December 2015
 * \author Jan Issac (jan.issac@gmail.com)
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#include <dbot/util/rigid_body_renderer.hpp>

#include <dbrt/visual_tracker.hpp>

namespace dbrt
{
VisualTracker::VisualTracker(
    const std::shared_ptr<Filter>& filter,
    const std::shared_ptr<dbot::ObjectModel>& object_model,
    int evaluation_count,
    int block_count)
    : object_model_(object_model),
      filter_(filter),
      evaluation_count_(evaluation_count),
      block_count_(block_count)
{
}

void VisualTracker::initialize(
    const std::vector<State>& initial_states)
{
    filter_->set_particles(initial_states);
    filter_->resample(evaluation_count_ / block_count_);
}

auto VisualTracker::track(const Obsrv& image) -> State
{
    filter_->filter(image, zero_input());

    State mean = filter_->belief().mean();

    return mean;
}
}
