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
 * \file invalid_number_of_joint_sigmas_exception.hpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <exception>

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


/**
 * \brief Represents an exception thrown if the number of indices in the
 * sampling block does not match the state dimension.
 */
class InvalidNumberOfSamplingBlocksException : public std::exception
{
public:
    const char* what() const noexcept
    {
        return "The number of indices in the sampling blocks does not match the "
               "number of joints (joint state dimension) of the robot.";
    }
};


class JointIndexOutOfBoundsException: public std::exception
{
public:
    const char* what() const noexcept
    {
        return "The the specified joint index is out of bounds of the number of"
               " of available joints!";
    }
};



}
