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
 * \file robot_rb_sensor_builder.cpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

// #include <kdl_parser/kdl_parser.hpp>
#include <dbrt/robot_state.h>

#include <dbot/builder/rb_sensor_builder.h>
#include <dbot/builder/rb_sensor_builder.hpp>

namespace dbot
{
template class RbSensorBuilder<dbrt::RobotState<>>;
}
