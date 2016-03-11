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
 * \file rbc_particle_filter_robot_tracker.cpp
 * \date December 2015
 * \author Jan Issac (jan.issac@gmail.com)
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#include <dbot/util/rigid_body_renderer.hpp>

#include <dbrt/rbc_particle_filter_robot_tracker.hpp>

namespace dbrt
{
RbcParticleFilterRobotTracker::RbcParticleFilterRobotTracker(
    const std::shared_ptr<Filter>& filter,
    const std::shared_ptr<dbot::ObjectModel>& object_model,
    const std::shared_ptr<dbot::CameraData>& camera_data,
    int evaluation_count)
    : object_model_(object_model),
      camera_data_(camera_data),
      filter_(filter),
      evaluation_count_(evaluation_count)
{
}

auto RbcParticleFilterRobotTracker::on_initialize(
    const std::vector<State>& initial_states,
    const Eigen::VectorXd& obsrv) -> State
{
    filter_->set_particles(initial_states);
    filter_->filter(obsrv, zero_input());

    // TODO determine what is the number of sampling blocks
    // eval_count / sampling blocks
    filter_->resample(evaluation_count_ / initial_states[0].size());

    State mean = filter_->belief().mean();
    return mean;
}

auto RbcParticleFilterRobotTracker::on_track(const Obsrv& image) -> State
{
    filter_->filter(image, zero_input());

    State mean = filter_->belief().mean();
    return mean;
}

const std::shared_ptr<dbot::CameraData>&
RbcParticleFilterRobotTracker::camera_data() const
{
    return camera_data_;
}
}
