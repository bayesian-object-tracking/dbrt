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
 * \file visual_tracker_node.hpp
 * \date December 2015
 * \author Jan Issac (jan.issac@gmail.com)
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#include <memory>

#include <cv.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>

#include <dbot/rigid_body_renderer.hpp>
#include <dbot_ros/util/ros_interface.hpp>
#include <dbot_ros/util/ros_camera_data_provider.hpp>

#include <dbrt/urdf_object_loader.h>
#include <dbrt/robot_state.hpp>
#include <dbrt/robot_publisher.h>
#include <dbrt/tracker/visual_tracker.h>
#include <dbrt/tracker/visual_tracker_ros.h>
#include <dbrt/tracker/visual_tracker_factory.h>
#include <dbrt/util/camera_data_factory.h>
#include <dbrt/util/kinematics_factory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_tracker");
    ros::NodeHandle nh("~");

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
    /* - Few types we will be using - */
    /* ------------------------------ */
    dbrt::RobotState<>::kinematics_ = kinematics;
    dbrt::RobotState<>::kinematics_mutex_ = std::make_shared<std::mutex>();
    typedef dbrt::RobotState<> State;

    /* ------------------------------ */
    /* - Initialize using joint msg - */
    /* ------------------------------ */
    sensor_msgs::JointState::ConstPtr joint_state;

    ros::NodeHandle nh_global;

    while (!joint_state)
    {
        ROS_INFO("Waiting for initial joint state");
        joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>(
            "/joint_states", nh_global, ros::Duration(1.));
    }

    /* ------------------------------ */
    /* - Create tracker             - */
    /* ------------------------------ */
    // parameter shorthand prefix
    std::string pre = "";
    auto tracker =
        dbrt::create_visual_tracker(pre, kinematics, camera_data, joint_state);
    dbrt::VisualTrackerRos tracker_ros(tracker, camera_data);

    /* ------------------------------ */
    /* - Tracker publisher          - */
    /* ------------------------------ */
    auto tracker_publisher = std::shared_ptr<dbrt::RobotPublisher<State>>(
        new dbrt::RobotPublisher<State>(
            kinematics,
            "/estimated",
            ri::read<std::string>("tf_connecting_frame", nh)));

    /* ------------------------------ */
    /* - Run tracker                - */
    /* ------------------------------ */
    ros::Subscriber subscriber =
        nh.subscribe(ri::read<std::string>("depth_image_topic", nh),
                     1,
                     &dbrt::VisualTrackerRos::track,
                     &tracker_ros);

    ROS_INFO("Running visual tracker");

    ros::Rate visualization_rate(100);

    while (ros::ok())
    {
        visualization_rate.sleep();
        auto current_state = tracker_ros.current_state();

        // std::cout << "PUBLISHING ESTIMATED JONT ANGLES AND TF WITH"
        //              "NOW() TIMESTAMP. THIS HAS TO BE FIXED!"
        //           << std::endl;
        ros::Time time = ros::Time::now();
        tracker_publisher->publish_tf(current_state, time);

        ros::spinOnce();
    }

    return 0;
}
