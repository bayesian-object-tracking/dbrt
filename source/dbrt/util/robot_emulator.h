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

/**
 * \file virtual_robot.hpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <chrono>
#include <dbot/camera_data.h>
#include <dbot/object_model.h>
#include <dbot/rigid_body_renderer.h>
#include <dbot_ros/util/ros_interface.h>
#include <dbrt/kinematics_from_urdf.h>
#include <dbrt/robot_publisher.h>
#include <dbrt/util/robot_animator.h>
#include <fl/util/profiling.hpp>
#include <functional>
#include <image_transport/image_transport.h>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/fill_image.h>
#include <thread>
#include <thread>

namespace dbrt
{
template <typename State>
class RobotEmulator
{
public:
    /**
     * \brief Creates a VirtualRobot
     */
    RobotEmulator(const std::shared_ptr<dbot::ObjectModel>& object_model,
                  const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics,
                  const std::shared_ptr<dbot::RigidBodyRenderer>& renderer,
                  const std::shared_ptr<dbot::CameraData>& camera_data,
                  const std::shared_ptr<RobotAnimator>& robot_animator,
                  double joint_sensors_rate,
                  double visual_sensor_rate,
                  double dilation,
                  double image_publishing_delay,
                  double image_timestamp_delay,
                  const State& initial_state)
        : time_(0.),
          state_(initial_state),
          object_model_(object_model),
          urdf_kinematics_(urdf_kinematics),
          renderer_(renderer),
          camera_data_(camera_data),
          robot_animator_(robot_animator),
          joint_sensors_rate_(joint_sensors_rate),
          visual_sensor_rate_(visual_sensor_rate),
          dilation_(dilation),
          image_publishing_delay_(image_publishing_delay),
          image_timestamp_delay_(image_timestamp_delay),
          paused_(false),
          node_handle_("~")
    {
        std::string prefix = "";

        robot_publisher_ = std::make_shared<RobotPublisher<State>>(
            urdf_kinematics_, prefix, "");

        // camera publisher
        pub_camera_info_ = node_handle_.advertise<sensor_msgs::CameraInfo>(
            prefix + "/XTION/depth/camera_info", 0);
        std::shared_ptr<image_transport::ImageTransport> it(
            new image_transport::ImageTransport(node_handle_));
        pub_depth_image_ = it->advertise(prefix + "/XTION/depth/image", 0);
    }

    void run()
    {
        running_ = true;
        joint_sensor_thread_ =
            std::thread(&RobotEmulator::run_joint_sensors, this);
        visual_sensor_thread_ =
            std::thread(&RobotEmulator::run_visual_sensors, this);
    }

    void pause() { paused_ = true; }
    void resume() { paused_ = false; }
    void toggle_pause() { paused_ = !paused_; }
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
        while (ros::ok() && running_)
        {
            while (ros::ok() && paused_)
            {
                usleep(1000);
            }

            joint_rate.sleep();
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            double delta_time = 1. / rate;
            robot_animator_->animate(state_, delta_time, dilation_, state_);
            time_ += delta_time;
            robot_publisher_->publish_joint_state(state_, ros::Time(time_));
            robot_publisher_->publish_tf(state_, ros::Time(time_));
        }
    }

    void run_visual_sensors()
    {
        ros::Rate image_rate(visual_sensor_rate_);
        while (running_)
        {
            while (ros::ok() && paused_)
            {
                usleep(1000);
            }

            image_rate.sleep();

            auto start = std::chrono::system_clock::now();
            State state;
            double acquisition_time;
            state_and_time(state, acquisition_time);

            double timestamp = acquisition_time + image_timestamp_delay_;

            Eigen::VectorXd depth_image;
            renderer_->Render(
                state, depth_image, std::numeric_limits<double>::quiet_NaN());

            auto end = std::chrono::system_clock::now();
            auto elapsed_time =
                std::chrono::duration<double>(end - start).count();

            /*
                        // publish in parallel
                        std::thread(
                                    [state,
                                    timestamp,
                                    elapsed_time,
                                    depth_image,
                                    image_publishing_delay_,
                                    &publisher_mutex_,
                                    &camera_data_,
                                    &robot_publisher_,
                                    this]()
                        {
                            double remaining_delay =
                                    std::max(image_publishing_delay_ -
               elapsed_time, 0.0);

                            ROS_INFO_STREAM("Visual simulation computation delay
               "
                                            << elapsed_time);

                            std::this_thread::sleep_for(
                                        std::chrono::duration<double>(remaining_delay));

                            std::lock_guard<std::mutex> publisher_lock(
                                        publisher_mutex_);
                            this->publish_camera_info(camera_data_,
                                                                  ros::Time(timestamp));
                            this->publish_image(
                                        depth_image, camera_data_,
               ros::Time(timestamp));

                        })
                                .detach();
            */
            std::thread(std::bind(&RobotEmulator<State>::publisher_thread,
                                  this,
                                  state,
                                  timestamp,
                                  elapsed_time,
                                  depth_image,
                                  image_publishing_delay_))
                .detach();
        }
    }

    void publisher_thread(State state,
                          double timestamp,
                          double elapsed_time,
                          Eigen::VectorXd depth_image,
                          double image_publishing_delay_)
    {
        double remaining_delay =
            std::max(image_publishing_delay_ - elapsed_time, 0.0);

        ROS_INFO_STREAM("Visual simulation computation delay " << elapsed_time);

        std::this_thread::sleep_for(
            std::chrono::duration<double>(remaining_delay));

        std::lock_guard<std::mutex> publisher_lock(publisher_mutex_);
        this->publish_camera_info(camera_data_, ros::Time(timestamp));
        this->publish_image(depth_image, camera_data_, ros::Time(timestamp));
    }

    void state_and_time(State& state, double& time) const
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        state = state_;
        time = time_;
    }

    Eigen::VectorXd joint_observation()
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        return state_;
    }

    //    const sensor_msgs::Image& observation()
    //    {
    //        std::lock_guard<std::mutex> image_lock(image_mutex_);
    //        return obsrv_image_;
    //    }

    //    const Eigen::VectorXd& observation_vector()
    //    {
    //        std::lock_guard<std::mutex> image_lock(image_mutex_);
    //        return obsrv_vector_;
    //    }

private:
    void convert_to_depth_image_msg(
        const std::shared_ptr<dbot::CameraData>& camera_data,
        const Eigen::VectorXd& depth_image,
        sensor_msgs::Image& image)
    {
        Eigen::VectorXf float_vector = depth_image.cast<float>();

        sensor_msgs::fillImage(image,
                               sensor_msgs::image_encodings::TYPE_32FC1,
                               camera_data->resolution().height,
                               camera_data->resolution().width,
                               camera_data->resolution().width * sizeof(float),
                               float_vector.data());
    }

    bool has_image_subscribers() const
    {
        return pub_depth_image_.getNumSubscribers() > 0;
    }

    void publish_image(const Eigen::VectorXd& depth_image,
                       const std::shared_ptr<dbot::CameraData>& camera_data,
                       const ros::Time& time)
    {
        if (pub_depth_image_.getNumSubscribers() > 0)
        {
            sensor_msgs::Image ros_image;
            convert_to_depth_image_msg(camera_data, depth_image, ros_image);

            ros_image.header.frame_id = camera_data->frame_id();
            ros_image.header.stamp = time;
            pub_depth_image_.publish(ros_image);
        }
    }

    void publish_camera_info(
        const std::shared_ptr<dbot::CameraData>& camera_data,
        const ros::Time& time)
    {
        auto camera_info = create_camera_info(camera_data, time);
        pub_camera_info_.publish(camera_info);
    }

    sensor_msgs::CameraInfoPtr create_camera_info(
        const std::shared_ptr<dbot::CameraData>& camera_data,
        const ros::Time& time)
    {
        sensor_msgs::CameraInfoPtr info_msg =
            boost::make_shared<sensor_msgs::CameraInfo>();

        info_msg->header.stamp = time;
        // if internal registration is used, rgb camera intrinsic parameters are
        // used
        info_msg->header.frame_id = camera_data->frame_id();
        info_msg->width = camera_data->native_resolution().width;
        info_msg->height = camera_data->native_resolution().height;

#if ROS_VERSION_MINIMUM(1, 3, 0)
        info_msg->D = std::vector<double>(5, 0.0);
        info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
#else
        info_msg->D.assign(0.0);
#endif
        info_msg->K.assign(0.0);
        info_msg->R.assign(0.0);
        info_msg->P.assign(0.0);

        auto camera_matrix = camera_data->camera_matrix();
        camera_matrix.topLeftCorner(2, 3) *=
            double(camera_data->downsampling_factor());

        for (size_t col = 0; col < 3; col++)
            for (size_t row = 0; row < 3; row++)
                info_msg->K[col + row * 3] = camera_matrix(row, col);

        // no rotation: identity
        info_msg->R[0] = info_msg->R[4] = info_msg->R[8] = 1.0;
        // no rotation, no translation => P=K(I|0)=(K|0)
        info_msg->P[0] = info_msg->P[5] = info_msg->K[0];
        info_msg->P[2] = info_msg->K[2];
        info_msg->P[6] = info_msg->K[5];
        info_msg->P[10] = 1.0;

        return info_msg;
    }

    double time_;
    Eigen::VectorXd state_;
    //    Eigen::VectorXd obsrv_vector_;
    sensor_msgs::Image obsrv_image_;
    std::shared_ptr<dbot::ObjectModel> object_model_;
    std::shared_ptr<KinematicsFromURDF> urdf_kinematics_;
    std::shared_ptr<dbot::RigidBodyRenderer> renderer_;
    std::shared_ptr<dbot::CameraData> camera_data_;
    std::shared_ptr<RobotAnimator> robot_animator_;
    std::shared_ptr<RobotPublisher<State>> robot_publisher_;
    double joint_sensors_rate_;
    double visual_sensor_rate_;
    double dilation_;
    double image_publishing_delay_;
    double image_timestamp_delay_;

    bool running_;
    mutable std::mutex state_mutex_;
    //    mutable std::mutex image_mutex_;
    mutable std::mutex publisher_mutex_;
    std::thread joint_sensor_thread_;
    std::thread visual_sensor_thread_;
    bool paused_;

    ros::NodeHandle node_handle_;
    ros::Publisher pub_camera_info_;
    image_transport::Publisher pub_depth_image_;
};
}
