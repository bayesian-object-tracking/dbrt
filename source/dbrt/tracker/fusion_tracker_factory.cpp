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
 * \file fusion_tracker_factory.cpp
 * \date July 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <string>

#include <memory>
#include <thread>
#include <functional>

#include <cv.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <fl/util/profiling.hpp>

#include <dbot/rigid_body_renderer.hpp>
#include <dbot/virtual_camera_data_provider.hpp>
#include <dbot/builder/particle_tracker_builder.hpp>

#include <dbot_ros/util/ros_interface.hpp>
#include <dbot_ros/util/data_set_camera_data_provider.hpp>

#include <dbrt/robot_state.hpp>
#include <dbrt/robot_publisher.h>
#include <dbrt/urdf_object_loader.h>
#include <dbrt/tracker/robot_tracker.h>
#include <dbrt/tracker/rotary_tracker.h>
#include <dbrt/tracker/visual_tracker.h>
#include <dbrt/tracker/fusion_tracker.h>

#include <dbrt/kinematics_from_urdf.h>
#include <dbrt/tracker/fusion_tracker_factory.h>
#include <dbrt/tracker/visual_tracker_factory.h>
#include <dbrt/tracker/rotary_tracker_factory.h>
#include <dbrt/builder/rotary_tracker_builder.hpp>

namespace dbrt
{
std::shared_ptr<dbrt::FusionTracker> create_fusion_tracker(
    const std::shared_ptr<KinematicsFromURDF>& kinematics,
    const std::shared_ptr<dbot::CameraData>& camera_data,
    sensor_msgs::JointState::ConstPtr joint_state)
{
    ros::NodeHandle nh("~");

    // parameter shorthand prefix
    std::string prefix = "";

    /* ------------------------------ */
    /* - Our state representation   - */
    /* ------------------------------ */
    typedef dbrt::RobotState<> State;

    /* ------------------------------ */
    /* - Initialize                 - */
    /* ------------------------------ */

    std::vector<Eigen::VectorXd> initial_states_vectors =
        {kinematics->sensor_msg_to_eigen(*joint_state)};
    std::vector<State> initial_states;
    for (auto state : initial_states_vectors)
    {
        initial_states.push_back(state);
    }

    /* ------------------------------ */
    /* - Create Tracker and         - */
    /* - tracker publisher          - */
    /* ------------------------------ */

    auto fusion_tracker = std::make_shared<dbrt::FusionTracker>(
        camera_data,
        kinematics,
        [=]()
        {
            return dbrt::create_rotary_tracker(prefix, kinematics, joint_state);
        },
        [=]()
        {
            return dbrt::create_visual_tracker(
                prefix, kinematics, camera_data, joint_state);
        },
        ri::read<double>(prefix + "camera_delay", nh));

    fusion_tracker->initialize(initial_states);

    return fusion_tracker;
}
}
