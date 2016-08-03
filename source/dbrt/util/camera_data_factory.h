/*
 * this is part of the bayesian object tracking (bot),
 * (https://github.com/bayesian-object-tracking)
 *
 * copyright (c) 2015 max planck society,
 * 				 autonomous motion department,
 * 			     institute for intelligent systems
 *
 * this source code form is subject to the terms of the gnu general public
 * license license (gnu gpl). a copy of the license can be found in the license
 * file distributed with this source code.
 */

/**
 * \file camera_data_factory.h
 * \date july 2016
 * \author jan issac (jan.issac@gmail.com)
 */

#pragma once

#include <memory>

#include <ros/ros.h>

#include <dbot/camera_data.hpp>

namespace dbrt
{
std::shared_ptr<dbot::CameraData> create_camera_data(ros::NodeHandle& nh);
}
