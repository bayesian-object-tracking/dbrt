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

#include <dbot_ros/util/ros_interface.hpp>
#include <dbot_ros/util/ros_camera_data_provider.hpp>

#include <dbrt/urdf_object_loader.h>

#include <dbrt/robot_state.hpp>
#include <dbrt/tracker/robot_tracker.h>
#include <dbot/rigid_body_renderer.hpp>
#include <dbrt/robot_publisher.h>
#include <dbrt/tracker/visual_tracker.h>

#include <dbrt/tracker/rotary_tracker_factory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotary_tracker");
    ros::NodeHandle nh("~");

    /* ------------------------------ */
    /* - Create the robot model     - */
    /* ------------------------------ */
    // initialize the kinematics
    std::shared_ptr<KinematicsFromURDF> urdf_kinematics(new KinematicsFromURDF(
        ri::read<std::string>("robot_description", ros::NodeHandle()),
        ri::read<std::string>("robot_description_package_path", nh),
        ri::read<std::string>("rendering_root_left", nh),
        ri::read<std::string>("rendering_root_right", nh),
        "NO_CAMERA_FRAME"));

    /* ------------------------------ */
    /* - Few types we will be using - */
    /* ------------------------------ */
    dbrt::RobotState<>::kinematics_ = urdf_kinematics;
    dbrt::RobotState<>::kinematics_mutex_ = std::make_shared<std::mutex>();

    // parameter shorthand prefix
    std::string pre = "rotary_tracker/";

    /* ------------------------------ */
    /* - Get initial joint values   - */
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
    auto tracker = dbrt::create_rotary_tracker(
        pre,
        urdf_kinematics->num_joints(),
        urdf_kinematics->GetJointOrder(*joint_state));

    /* ------------------------------ */
    /* - Initialize tracker         - */
    /* ------------------------------ */
    std::vector<Eigen::VectorXd> initial_states_vectors =
        urdf_kinematics->GetInitialJoints(*joint_state);
    std::vector<dbrt::RobotState<>> initial_states;
    for (auto state : initial_states_vectors) initial_states.push_back(state);
    tracker->initialize(initial_states);

    /* ------------------------------ */
    /* - tracker publisher          - */
    /* ------------------------------ */
    auto tracker_publisher =
        std::make_shared<dbrt::RobotTrackerPublisher<dbrt::RobotState<>>>(
            urdf_kinematics,
            "/estimated",
            "Base");
    /* ------------------------------ */
    /* - Run tracker                - */
    /* ------------------------------ */
    ROS_INFO("Running rotary tracker");

    ros::Subscriber subscriber = nh.subscribe(
        "/joint_states", 1, &dbrt::RotaryTracker::track_callback, &(*tracker));

    ros::Rate visualization_rate(100);
    while (ros::ok())
    {
        visualization_rate.sleep();
        auto current_state = tracker->current_state();

        /// \todo: THIS IS A HACK!! WE SHOULD PASS THE PROPER TIME WHICH
        /// CORRESPONDS TO THE MEASUREMENT
        std::cout << "PUBLISHING ESTIMATED JONT ANGLES AND TF WITH"
                     "NOW() TIMESTAMP. THIS HAS TO BE FIXED!"
                  << std::endl;
        ros::Time time = ros::Time::now();
        tracker_publisher->publish_tf(current_state, time);

        ros::spinOnce();
    }

    ros::spin();

    return 0;
}
