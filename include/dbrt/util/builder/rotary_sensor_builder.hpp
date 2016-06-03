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
 * \file robot_joint_observation_model_builder.hpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <memory>

#include <fl/util/profiling.hpp>
#include <fl/util/meta.hpp>
#include <fl/model/observation/linear_gaussian_observation_model.hpp>

#include <Eigen/Dense>

#include <dbot/builder/state_transition_function_builder.hpp>

#include <dbrt/util/builder/exceptions.hpp>
#include <dbrt/util/builder/exceptions.hpp>

namespace dbrt
{

template <typename Tracker>
class RotarySensorBuilder
{
public:
    enum Dimension
    {
        StateDim = Tracker::JointStateDim,
        ObsrvDim = Tracker::JointObsrvDim
    };

    typedef typename Tracker::JointState State;
    typedef typename Tracker::JointObsrv Obsrv;
    typedef typename Tracker::JointObsrvModel Model;

    struct Parameters
    {
        std::vector<double> joint_sigmas;
        int joint_count;
    };

    RotarySensorBuilder(const Parameters& param) : param_(param)
    {
    }

    virtual std::shared_ptr<Model> build(int joint_index) const
    {
        if (param_.joint_count != param_.joint_sigmas.size())
        {
            throw InvalidNumberOfJointSigmasException();
        }

        if (joint_index < 0 || joint_index >= param_.joint_count)
        {
            throw JointIndexOutOfBoundsException();
        }


        if(StateDim != 2 || ObsrvDim != 1)
        {
            std::cout << "dawg you screwed up dimensions" << std::endl;
            exit(-1);
        }

        auto model = std::make_shared<Model>(ObsrvDim, StateDim);

        auto H = model->create_sensor_matrix();
        auto R = model->create_noise_matrix();

        H.setOnes();
        R.setIdentity();
        R *= param_.joint_sigmas[joint_index];

        model->sensor_matrix(H);
        model->noise_matrix(R);

        return model;
    }

private:
    Parameters param_;
};
}
