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
 * \file gaussian_joint_robot_tracker.cpp
 * \date March 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <dbrt/gaussian_joint_filter_robot_tracker.hpp>

namespace dbrt
{
GaussianJointFilterRobotTracker::GaussianJointFilterRobotTracker(
    const std::shared_ptr<std::vector<JointFilter>>& joint_filters)
    : joint_filters_(joint_filters)
{
}

const std::vector<GaussianJointFilterRobotTracker::JointBelief>&
GaussianJointFilterRobotTracker::beliefs() const
{
    return beliefs_;
}

std::vector<GaussianJointFilterRobotTracker::JointBelief>&
GaussianJointFilterRobotTracker::beliefs()
{
    return beliefs_;
}

auto GaussianJointFilterRobotTracker::on_initialize(
    const std::vector<State>& initial_states,
    const Eigen::VectorXd& obsrv) -> State
{
    const int dim_joint = JointObsrv::SizeAtCompileTime;
    State state;
    state.resize(dim_joint * joint_filters_->size());

    beliefs_.resize(joint_filters_->size());

    for (int i = 0; i < joint_filters_->size(); ++i)
    {
        beliefs_[i] = (*joint_filters_)[i].create_belief();

        auto cov = beliefs_[i].covariance();
        cov.setZero();
        beliefs_[i].covariance(cov);
        beliefs_[i].mean(
            initial_states[0].middleRows(i * dim_joint, dim_joint));

        state.middleRows(i * dim_joint, dim_joint) = beliefs_[i].mean();
    }

    return state;
}

auto GaussianJointFilterRobotTracker::on_track(const Obsrv& joints_obsrv)
    -> State
{
//    INIT_PROFILING
    const int dim_joint = JointObsrv::SizeAtCompileTime;
    State state;
    state.resize(dim_joint * joint_filters_->size());

    for (int i = 0; i < joint_filters_->size(); ++i)
    {
        (*joint_filters_)[i].predict(
            beliefs_[i], JointInput::Zero(), beliefs_[i]);

        (*joint_filters_)[i].update(
            beliefs_[i],
            joints_obsrv.middleRows(i * dim_joint, dim_joint),
            beliefs_[i]);

        state.middleRows(i * dim_joint, dim_joint) = beliefs_[i].mean();
    }
//    MEASURE_FLUSH("Track");

    return state;
}
}
