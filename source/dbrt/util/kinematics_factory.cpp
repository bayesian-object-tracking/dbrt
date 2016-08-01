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
 * \file kinematics_factory.cpp
 * \date July 2016
 * \author jan issac (jan.issac@gmail.com)
 */

#include <dbrt/util/kinematics_factory.h>

#include <dbot_ros/util/ros_interface.hpp>

namespace dbrt
{
std::shared_ptr<KinematicsFromURDF> create_kinematics(
    ros::NodeHandle& nh,
    const std::string& camera_frame_id)
{
    auto robot_description = ri::read<std::string>(
        ri::read<std::string>("robot_description_name", nh), ros::NodeHandle());
    auto robot_description_package_path =
        ri::read<std::string>("robot_description_package_path", nh);
    auto rendering_root_left = ri::read<std::string>("rendering_root_left", nh);
    auto rendering_root_right =
        ri::read<std::string>("rendering_root_right", nh);

    auto estimate_camera_offset = ri::read<bool>("estimate_camera_offset", nh);

    std::string prefixed_frame_id = camera_frame_id;
    std::size_t slash_index = prefixed_frame_id.find_last_of("/");
    std::string frame_id = prefixed_frame_id.substr(slash_index + 1);

    return std::make_shared<KinematicsFromURDF>(robot_description,
                                                robot_description_package_path,
                                                rendering_root_left,
                                                rendering_root_right,
                                                frame_id,
                                                estimate_camera_offset);
}
}
