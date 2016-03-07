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
 * \file joint_index_out_of_bounds_exception.hpp
 * \date February 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <exception>

namespace dbrt
{

/**
 * \brief Represents an exception thrown if the specified joint index is out of
 * out of bounds of the existing joints of a robot
 */
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
