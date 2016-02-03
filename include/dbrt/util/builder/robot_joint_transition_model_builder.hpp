/*
 * This is part of the Bayesian Robot Tracking (brt),
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
 * \file robot_joint_transition_model_builder.hpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <memory>

#include <fl/util/profiling.hpp>
#include <fl/util/meta.hpp>

#include <Eigen/Dense>

#include <dbot/tracker/builder/state_transition_function_builder.hpp>
#include <fl/model/process/linear_state_transition_model.hpp>

namespace dbrt
{

/**
 * \brief Represents an exception thrown if the number of joint sigmas in the
 * of the joint_transition model does not match the state dimension.
 */
class InvalidNumberOfJointSigmasException : public std::exception
{
public:
    const char* what() const noexcept
    {
        return "The number of joint sigmas does not match the number of joints "
               "of the robot!";
    }
};

template <typename State>
struct RobotJointStateTrait
{
    enum
    {
        NoiseDim = State::SizeAtCompileTime != -1 ? State::SizeAtCompileTime / 2
                                                  : Eigen::Dynamic,
        InputDim = Eigen::Dynamic
    };

    typedef Eigen::Matrix<typename State::Scalar, NoiseDim, 1> Noise;
    typedef Eigen::Matrix<typename State::Scalar, InputDim, 1> Input;
};

template <typename State>
class RobotJointTransitionModelBuilder
    : public dbot::StateTransitionFunctionBuilder<
          State,
          typename RobotJointStateTrait<State>::Noise,
          typename RobotJointStateTrait<State>::Input>
{
public:
    typedef fl::StateTransitionFunction<
        State,
        typename RobotJointStateTrait<State>::Noise,
        typename RobotJointStateTrait<State>::Input> Model;
    typedef fl::LinearStateTransitionModel<
        State,
        typename RobotJointStateTrait<State>::Noise,
        typename RobotJointStateTrait<State>::Input> DerivedModel;

    struct Parameters
    {
        double joint_sigma;
        std::vector<double> joint_sigmas;
        int joint_count;
    };

    RobotJointTransitionModelBuilder(const Parameters& param) : param_(param) {}
    virtual std::shared_ptr<Model> build() const
    {
        auto model =
            std::shared_ptr<DerivedModel>(new DerivedModel(build_model()));

        return std::static_pointer_cast<Model>(model);
    }

    virtual DerivedModel build_model() const
    {
        if (param_.joint_count != param_.joint_sigmas.size())
        {
            throw InvalidNumberOfJointSigmasException();
        }

        int total_state_dim = param_.joint_count;
        int total_noise_dim = total_state_dim;

        auto model = DerivedModel(total_state_dim, total_noise_dim, 1);

        auto A = model.create_dynamics_matrix();
        auto B = model.create_noise_matrix();
        auto C = model.create_input_matrix();

        auto part_A = Eigen::Matrix<fl::Real, 1, 1>();
        auto part_B = Eigen::Matrix<fl::Real, 1, 1>();

        A.setZero();
        B.setZero();
        C.setZero();

        part_A.setIdentity();

        part_B.setIdentity();
        part_B *= param_.joint_sigma;

        for (int i = 0; i < param_.joint_count; ++i)
        {
            A.block(i, i, 1, 1) = part_A;
            B(i, i) = param_.joint_sigmas[i];
        }

        model.dynamics_matrix(A);
        model.noise_matrix(B);
        model.input_matrix(C);

        PV(model.dynamics_matrix());
        PV(model.noise_matrix());

        return model;
    }

private:
    Parameters param_;
};
}
