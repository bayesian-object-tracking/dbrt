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
 * \file camera_data_factory.cpp
 * \date July 2016
 * \author jan issac (jan.issac@gmail.com)
 */

#include <dbot_ros/util/ros_camera_data_provider.h>
#include <dbot_ros/util/ros_interface.h>
#include <dbrt/util/camera_data_factory.h>

namespace dbrt
{
std::shared_ptr<dbot::CameraData> create_camera_data(ros::NodeHandle& nh)
{
    auto camera_info_topic = ri::read<std::string>("camera_info_topic", nh);
    auto depth_image_topic = ri::read<std::string>("depth_image_topic", nh);
    auto downsampling_factor = ri::read<int>("downsampling_factor", nh);
    dbot::CameraData::Resolution resolution;
    resolution.width = ri::read<int>("resolution/width", nh);
    resolution.height = ri::read<int>("resolution/height", nh);

    return std::make_shared<dbot::CameraData>(
        std::make_shared<dbot::RosCameraDataProvider>(nh,
                                                      camera_info_topic,
                                                      depth_image_topic,
                                                      resolution,
                                                      downsampling_factor,
                                                      2.0));
}
}
