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
 * \file fusion_tracker_node.hpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

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
#include <dbot/builder/rbc_particle_filter_tracker_builder.hpp>

#include <dbot_ros/util/ros_interface.hpp>
#include <dbot_ros/util/data_set_camera_data_provider.hpp>

#include <dbrt/robot_state.hpp>
#include <dbrt/robot_publisher.h>
#include <dbrt/urdf_object_loader.h>
#include <dbrt/tracker/robot_tracker.h>
#include <dbrt/tracker/rotary_tracker.h>
#include <dbrt/tracker/visual_tracker.h>
#include <dbrt/tracker/fusion_tracker.h>
#include <dbrt/tracker/fusion_tracker_factory.h>

#include <dbrt/util/camera_data_factory.h>
#include <dbrt/util/kinematics_factory.h>

/**
 * \brief Node entry point
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_tracker");
    ros::NodeHandle nh("~");

    typedef dbrt::RobotState<> State;

    /* ------------------------------ */
    /* - Setup camera data          - */
    /* ------------------------------ */
    auto camera_data = dbrt::create_camera_data(nh);

    /* ------------------------------ */
    /* - Create the robot kinematics- */
    /* - and robot mesh model       - */
    /* ------------------------------ */
    auto kinematics = dbrt::create_kinematics(nh, camera_data->frame_id());

    /* ------------------------------ */
    /* - Initial states               */
    /* ------------------------------ */

    ros::NodeHandle nh_global;
    sensor_msgs::JointState::ConstPtr init_joint_state;
    while (!init_joint_state)
    {
        ROS_INFO("Waiting for initial joint state");
        init_joint_state =
            ros::topic::waitForMessage<sensor_msgs::JointState>(
                "/joint_states", nh_global, ros::Duration(2.));
    }

    /* ------------------------------ */
    /* - Create Tracker               */
    /* ------------------------------ */
    auto fusion_tracker = dbrt::create_fusion_tracker(
        kinematics, camera_data, init_joint_state);

    /* ------------------------------ */
    /* - Tracker publisher          - */
    /* ------------------------------ */
    auto tracker_publisher = std::make_shared<dbrt::RobotPublisher<State>>(
        kinematics,
        "/estimated",
        ri::read<std::string>("tf_connecting_frame", nh));

    /* ------------------------------ */
    /* - Run tracker node           - */
    /* ------------------------------ */
    fusion_tracker->run();

    ros::Subscriber joint_subscriber =
        nh.subscribe("/joint_states",
                     1000,
                     &dbrt::FusionTracker::joints_obsrv_callback,
                     fusion_tracker.get());

    ros::Subscriber image_subscriber =
        nh.subscribe(ri::read<std::string>("depth_image_topic", nh),
                     1,
                     &dbrt::FusionTracker::image_obsrv_callback,
                     fusion_tracker.get());

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate visualization_rate(100);
    while (ros::ok())
    {
        visualization_rate.sleep();

        State current_state;
        double current_time;
        dbrt::JointsObsrv current_angle_measurement;
        fusion_tracker->current_things(
            current_state, current_time, current_angle_measurement);

        if (current_angle_measurement.size() > 0)
        {
            tracker_publisher->publish_tf(current_state,
                                          current_angle_measurement,
                                          "",
                                          ros::Time(current_time));
        }

        ros::spinOnce();
    }

    ROS_INFO("Shutting down ...");
    fusion_tracker->shutdown();

    return 0;
}
