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
 * \file virtual_robot.hpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <memory>

#include <sensor_msgs/Image.h>

#include <dbot/util/camera_data.hpp>
#include <dbot/util/object_model.hpp>
#include <dbot/util/rigid_body_renderer.hpp>

#include <dbrt/robot_tracker_publisher.h>
#include <dbrt/util/kinematics_from_urdf.hpp>

namespace dbrt
{
template <typename State>
class VirtualRobot
{
public:
public:
    /**
     * \brief Creates a VirtualRobot
     */
    VirtualRobot(const std::shared_ptr<dbot::ObjectModel>& object_model,
                 const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics,
                 const std::shared_ptr<dbot::RigidBodyRenderer>& renderer,
                 const std::shared_ptr<dbot::CameraData>& camera_data)
        : object_model_(object_model),
          urdf_kinematics_(urdf_kinematics),
          renderer_(renderer),
          camera_data_(camera_data)
    {
        t = 0.0;
        robot_tracker_publisher_simulated_ =
            std::make_shared<RobotTrackerPublisher<State>>(
                urdf_kinematics_, renderer_, "/simulated");
//        robot_tracker_publisher_estimated_ =
//            std::make_shared<RobotTrackerPublisher<State>>(
//                urdf_kinematics_, renderer_, "/estimated");
    }

    void publish(State state)
    {

        for (int i = 6; i < 6 + 7; ++i)
        {
            state[i] += 0.1 * std::sin(t * 10.);
        }

        for (int i = 6 + 7 + 8; i < 6 + 2 * 7 + 8; ++i)
        {
            state[i] += 0.1 * std::sin(t * 10.);
        }

        robot_tracker_publisher_simulated_->publish(
            state, sensor_msgs::Image(), camera_data_);


//        for (int i = 6; i < 6 + 7; ++i)
//        {
//            state[i] += 0.1 * std::sin(t * 10.);
//        }

//        for (int i = 6 + 7 + 8; i < 6 + 2 * 7 + 8; ++i)
//        {
//            state[i] += 0.1 * std::sin(t * 10.);
//        }


        t += 1.e-3;

//        robot_tracker_publisher_estimated_->publish(
//            state, sensor_msgs::Image(), camera_data_);
    }

private:
    double t;
    std::shared_ptr<dbot::ObjectModel> object_model_;
    std::shared_ptr<KinematicsFromURDF> urdf_kinematics_;
    std::shared_ptr<dbot::RigidBodyRenderer> renderer_;
    std::shared_ptr<dbot::CameraData> camera_data_;
    std::shared_ptr<RobotTrackerPublisher<State>> robot_tracker_publisher_simulated_;
    std::shared_ptr<RobotTrackerPublisher<State>> robot_tracker_publisher_estimated_;
};
}
