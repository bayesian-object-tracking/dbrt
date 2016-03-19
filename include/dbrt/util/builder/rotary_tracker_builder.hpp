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

#include <dbrt/util/kinematics_from_urdf.hpp>
#include <dbrt/gaussian_joint_filter_robot_tracker.hpp>
#include <dbrt/util/builder/robot_joint_transition_model_builder.hpp>
#include <dbrt/util/builder/robot_joint_observation_model_builder.hpp>

namespace dbrt
{
template <typename Tracker>
class RotaryTrackerBuilder
{
public:
    typedef typename Tracker::State State;
    typedef typename Tracker::Noise Noise;
    typedef typename Tracker::Input Input;
    typedef typename Tracker::JointFilter JointFilter;

public:
    RotaryTrackerBuilder(
        const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics,
        const std::shared_ptr<FactorizedTransitionModelBuilder<Tracker>>&
            state_transition_builder,
        const std::shared_ptr<RotaryObsrvModelBuilder<Tracker>>&
            obsrv_model_builder)
        : state_transition_builder_(state_transition_builder),
          obsrv_model_builder_(obsrv_model_builder),
          urdf_kinematics_(urdf_kinematics)
    {
    }

    /**
     * \brief Builds the Rbc PF tracker
     */
    std::shared_ptr<Tracker> build()
    {
        auto joint_filters = create_joint_filters();

        auto tracker = std::make_shared<Tracker>(joint_filters);

        return tracker;
    }

    virtual std::shared_ptr<std::vector<JointFilter>> create_joint_filters()
    {
        auto joint_filters = std::make_shared<std::vector<JointFilter>>();

        for (int i = 0; i < urdf_kinematics_->num_joints(); ++i)
        {
            auto state_transition_model =
                this->state_transition_builder_->build(i);
            auto obsrv_model = this->obsrv_model_builder_->build(i);

            auto filter = JointFilter(*state_transition_model, *obsrv_model);

            joint_filters->push_back(filter);
        }
        return joint_filters;
    }

protected:
    std::shared_ptr<FactorizedTransitionModelBuilder<Tracker>>
        state_transition_builder_;
    std::shared_ptr<RotaryObsrvModelBuilder<Tracker>>
        obsrv_model_builder_;
    std::shared_ptr<KinematicsFromURDF> urdf_kinematics_;
};
}
