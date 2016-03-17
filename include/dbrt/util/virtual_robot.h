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
#include <thread>
#include <functional>

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
    typedef std::function<void(const Eigen::MatrixXd&)> JointSensorCallback;
    typedef std::function<void(const Eigen::MatrixXd&)> ImageSensorCallback;

public:
    /**
     * \brief Creates a VirtualRobot
     */
    VirtualRobot(const std::shared_ptr<dbot::ObjectModel>& object_model,
                 const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics,
                 const std::shared_ptr<dbot::RigidBodyRenderer>& renderer,
                 const std::shared_ptr<dbot::CameraData>& camera_data,
                 double joint_sensors_rate,
                 double visual_sensor_rate,
                 const State& initial_state)
        : state_(initial_state),
          object_model_(object_model),
          urdf_kinematics_(urdf_kinematics),
          renderer_(renderer),
          camera_data_(camera_data),
          joint_sensors_rate_(joint_sensors_rate),
          visual_sensor_rate_(visual_sensor_rate)
    {
        t = 0.0;
        robot_tracker_publisher_simulated_ =
            std::make_shared<RobotTrackerPublisher<State>>(
                urdf_kinematics_, renderer_, "/sensors");

        render_and_publish();
    }

    void run()
    {
        running_ = true;
        joint_sensor_thread_ =
            std::thread(&VirtualRobot::run_joint_sensors, this);
        visual_sensor_thread_ =
            std::thread(&VirtualRobot::run_visual_sensors, this);
    }

    void shutdown()
    {
        running_ = false;
        joint_sensor_thread_.join();
        visual_sensor_thread_.join();
    }

    void run_joint_sensors()
    {
        const double rate = joint_sensors_rate_;
        ros::Rate joint_rate(rate);
        while (running_)
        {
            joint_rate.sleep();
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            for (int i = 6; i < 6 + 7; ++i)
            {
                state_[i] += 0.1 / rate * std::sin(t);
            }

            for (int i = 6 + 7 + 8; i < 6 + 2 * 7 + 8; ++i)
            {
                state_[i] += 0.1 / rate * std::sin(t);
            }

            t += 1. / rate;

            if (joint_sensor_callback_)
            {
                joint_sensor_callback_(state_);
            }
        }
    }

    void run_visual_sensors()
    {
        ros::Rate image_rate(visual_sensor_rate_);
        while (running_)
        {
            image_rate.sleep();
            render_and_publish();

            if (image_sensor_callback_)
            {
                image_sensor_callback_(obsrv_vector_);
            }
        }
    }

    State state() const
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        return state_;
    }

    Eigen::VectorXd joint_observation()
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        return state_;
    }

    sensor_msgs::Image& observation()
    {
        std::lock_guard<std::mutex> image_lock(image_mutex_);
        return obsrv_image_;
    }

    Eigen::VectorXd& observation_vector()
    {
        std::lock_guard<std::mutex> image_lock(image_mutex_);
        return obsrv_vector_;
    }

    void joint_sensor_callback(const JointSensorCallback& callback)
    {
        joint_sensor_callback_ = callback;
    }

    void image_sensor_callback(const ImageSensorCallback& callback)
    {
        image_sensor_callback_ = callback;
    }

private:
    void render_and_publish()
    {
        State state;
        {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            state = state_;
        }

        {
            std::lock_guard<std::mutex> image_lock(image_mutex_);
            // render observation image
            renderer_->Render(
                state, obsrv_vector_, std::numeric_limits<double>::quiet_NaN());
        }

        // convert image vector to ros image message
        robot_tracker_publisher_simulated_->convert_to_depth_image_msg(
            camera_data_, obsrv_vector_, obsrv_image_);

        // publish observation image and point cloud
        robot_tracker_publisher_simulated_->publish(
            state, obsrv_image_, camera_data_);
    }

private:
    double t;
    Eigen::VectorXd state_;
    Eigen::VectorXd obsrv_vector_;
    sensor_msgs::Image obsrv_image_;
    std::shared_ptr<dbot::ObjectModel> object_model_;
    std::shared_ptr<KinematicsFromURDF> urdf_kinematics_;
    std::shared_ptr<dbot::RigidBodyRenderer> renderer_;
    std::shared_ptr<dbot::CameraData> camera_data_;
    std::shared_ptr<RobotTrackerPublisher<State>>
        robot_tracker_publisher_simulated_;
    double joint_sensors_rate_;
    double visual_sensor_rate_;

    bool running_;
    mutable std::mutex state_mutex_;
    mutable std::mutex image_mutex_;
    std::thread joint_sensor_thread_;
    std::thread visual_sensor_thread_;
    JointSensorCallback joint_sensor_callback_;
    ImageSensorCallback image_sensor_callback_;
};
}
