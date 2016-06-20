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
 * \file rotaty_tracker_factoy.hpp
 * \date June 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <string>
#include <vector>

#include <dbrt/rotary_tracker.hpp>

namespace dbrt
{
    
/**
 * \brief Create a gaussian filter tracking the robot joints based on joint
 *     measurements
 * \param prefix
 *     parameter prefix, e.g. fusion_tracker
 * \param kinematics
 *     URDF robot kinematics
 */
std::shared_ptr<dbrt::RotaryTracker> create_rotary_tracker(
    const std::string& prefix,
    const int& joint_count,
    const std::vector<int>& joint_order);

}
