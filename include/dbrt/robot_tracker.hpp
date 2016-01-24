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
 * \file robot_tracker.hpp
 * \date December 2015
 * \author Jan Issac (jan.issac@gmail.com)
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#pragma once

#include <Eigen/Dense>

#include <vector>
#include <string>
#include <memory>
#include <mutex>

#include <dbot/util/camera_data.hpp>
#include <dbot/util/object_model.hpp>

#include <osr/pose_vector.hpp>
#include <osr/composed_vector.hpp>
#include <osr/free_floating_rigid_bodies_state.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <robot_state_pub/robot_state_publisher.h>

#include <dbrt/robot_state.hpp>
#include <dbrt/util/kinematics_from_urdf.hpp>

namespace dbrt
{
/**
 * \brief Abstract RobotTracker context
 */
class RobotTracker
{
public:
    typedef RobotState<> State;
    typedef Eigen::Matrix<fl::Real, Eigen::Dynamic, 1> Obsrv;
    typedef Eigen::Matrix<fl::Real, Eigen::Dynamic, 1> Noise;
    typedef Eigen::Matrix<fl::Real, Eigen::Dynamic, 1> Input;

public:
    /**
     * \brief Creates the tracker
     *
     * \param filter
     *     Rbc particle filter instance
     * \param object_model
     *     Object model instance
     * \param camera_data
     *     Camera data container
     * \param update_rate
     *     Moving average update rate
     */
    RobotTracker(const std::shared_ptr<dbot::ObjectModel>& object_model,
                 const std::shared_ptr<dbot::CameraData>& camera_data);

    /**
     * \brief Hook function which is called during tracking
     * \return Current belief state
     */
    virtual State on_track(const Obsrv& image) = 0;

    /**
     * \brief Hook function which is called during initialization
     * \return Initial belief state
     */
    virtual State on_initialize(
        const std::vector<State>& initial_states,
        std::shared_ptr<KinematicsFromURDF>& urdf_kinematics) = 0;

    /**
     * \brief perform a single filter step
     *
     * \param image
     *     Current observation image
     */
    State track(const Obsrv& image);

    /**
     * \brief Initializes the particle filter with the given initial states and
     *     the number of evaluations
     * @param initial_states
     * @param evaluation_count
     */
    void initialize(const std::vector<State>& initial_states,
                    std::shared_ptr<KinematicsFromURDF>& urdf_kinematics);

    /**
     * \brief Returns camera data
     */
    const std::shared_ptr<dbot::CameraData> &camera_data() const;

    /**
     * \brief Shorthand for a zero input vector
     */
    Input zero_input() const;

protected:
    std::shared_ptr<dbot::ObjectModel> object_model_;
    std::shared_ptr<dbot::CameraData> camera_data_;
    std::mutex mutex_;
};
}
