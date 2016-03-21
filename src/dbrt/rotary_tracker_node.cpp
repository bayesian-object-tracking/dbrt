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

#include <dbot_ros/tracker_node.h>
#include <dbot_ros/tracker_publisher.h>
#include <dbot_ros/utils/ros_interface.hpp>
#include <dbot_ros/utils/ros_camera_data_provider.hpp>

#include <dbrt/util/urdf_object_loader.hpp>

#include <dbrt/robot_state.hpp>
#include <dbrt/robot_tracker.hpp>
#include <dbot/util/rigid_body_renderer.hpp>
#include <dbrt/robot_publisher.h>
#include <dbrt/visual_tracker.hpp>

#include <dbrt/util/builder/rotary_tracker_builder.hpp>

/**
 * \brief Create a gaussian filter tracking the robot joints based on joint
 *     measurements
 * \param prefix
 *     parameter prefix, e.g. fusion_tracker
 * \param kinematics
 *     URDF robot kinematics
 */
std::shared_ptr<dbrt::RotaryTracker> create_rotary_tracker(
    const std::string& prefix,
    const int& joint_count)
{
    ros::NodeHandle nh("~");

    typedef dbrt::RotaryTracker Tracker;

    /* ------------------------------ */
    /* - State transition function  - */
    /* ------------------------------ */
    dbrt::FactorizedTransitionBuilder<Tracker>::Parameters
        transition_parameters;

    // linear state transition parameters
    nh.getParam(prefix + "joint_transition/joint_sigmas",
                transition_parameters.joint_sigmas);
    nh.getParam(prefix + "joint_transition/bias_sigmas",
                transition_parameters.bias_sigmas);
    nh.getParam(prefix + "joint_transition/bias_factors",
                transition_parameters.bias_factors);
    transition_parameters.joint_count = joint_count;

    auto transition_builder =
        std::make_shared<dbrt::FactorizedTransitionBuilder<Tracker>>(
            (transition_parameters));

    /* ------------------------------ */
    /* - Observation model          - */
    /* ------------------------------ */
    dbrt::RotarySensorBuilder<Tracker>::Parameters sensor_parameters;

    nh.getParam(prefix + "joint_observation/joint_sigmas",
                sensor_parameters.joint_sigmas);
    sensor_parameters.joint_count = joint_count;

    PV(joint_count);

    auto rotary_sensor_builder =
        std::make_shared<dbrt::RotarySensorBuilder<Tracker>>(sensor_parameters);

    /* ------------------------------ */
    /* - Build the tracker          - */
    /* ------------------------------ */
    auto tracker_builder = dbrt::RotaryTrackerBuilder<Tracker>(
        joint_count, transition_builder, rotary_sensor_builder);

    return tracker_builder.build();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotary_tracker");
    ros::NodeHandle nh("~");

    /* ---------------------------------------------------------------------- */
    /* Roa-Blackwellized Coordinate Particle Filter Robot Tracker             */
    /*                                                                        */
    /* Ingredients:                                                           */
    /*   - TrackerNode                                                        */
    /*     - Tracker                                                          */
    /*       - Rbc Particle Filter Algorithm                                  */
    /*         - Robot state transition model                                 */
    /*         - Observation model                                            */
    /*       - Object model                                                   */
    /*       - Camera data                                                    */
    /*     - Tracker publisher to advertise the estimated state               */
    /*                                                                        */
    /*  Construnction of the tracker will utilize few builders and factories. */
    /*  For that, we need the following builders/factories:                   */
    /*    - Robot state transition model builder                              */
    /*    - Observation model builder to build GPU or CPU based models        */
    /*    - Filter builder                                                    */
    /* ---------------------------------------------------------------------- */

    /* ------------------------------ */
    /* - Create the robot model     - */
    /* ------------------------------ */
    // initialize the kinematics
    std::shared_ptr<KinematicsFromURDF> urdf_kinematics(
        new KinematicsFromURDF());

    auto object_model_loader = std::shared_ptr<dbot::ObjectModelLoader>(
        new dbrt::UrdfObjectModelLoader(urdf_kinematics));

    // Load the model usign the URDF loader
    auto object_model =
        std::make_shared<dbot::ObjectModel>(object_model_loader, false);

    /* ------------------------------ */
    /* - Setup camera data          - */
    /* ------------------------------ */
    int downsampling_factor;
    std::string camera_info_topic;
    std::string depth_image_topic;
    dbot::CameraData::Resolution resolution;
    nh.getParam("camera_info_topic", camera_info_topic);
    nh.getParam("depth_image_topic", depth_image_topic);
    nh.getParam("downsampling_factor", downsampling_factor);
    nh.getParam("resolution/width", resolution.width);
    nh.getParam("resolution/height", resolution.height);

    auto camera_data_provider = std::shared_ptr<dbot::CameraDataProvider>(
        new dbot::RosCameraDataProvider(nh,
                                        camera_info_topic,
                                        depth_image_topic,
                                        resolution,
                                        downsampling_factor,
                                        2.0));
    // Create camera data from the RosCameraDataProvider which takes the data
    // from a ros camera topic
    auto camera_data = std::make_shared<dbot::CameraData>(camera_data_provider);

    /* ------------------------------ */
    /* - Few types we will be using - */
    /* ------------------------------ */
    dbrt::RobotState<>::kinematics_ = urdf_kinematics;
    typedef dbrt::RobotState<> State;

    typedef dbrt::RotaryTracker Tracker;

    // parameter shorthand prefix
    std::string pre = "rotary_tracker/";

    //    urdf_kinematics->InitKDLData(Eigen::VectorXd::Zero(urdf_kinematics->num_joints()));
    auto tracker = create_rotary_tracker(pre, urdf_kinematics->num_joints());

    /* ------------------------------ */
    /* - Tracker publisher          - */
    /* ------------------------------ */

    std::shared_ptr<dbot::RigidBodyRenderer> renderer(
        new dbot::RigidBodyRenderer(object_model->vertices(),
                                    object_model->triangle_indices(),
                                    camera_data->camera_matrix(),
                                    camera_data->resolution().height,
                                    camera_data->resolution().width));

    auto tracker_publisher = std::shared_ptr<dbrt::RobotTrackerPublisher<State>>(
        new dbrt::RobotTrackerPublisher<State>(
            urdf_kinematics, renderer, "/estimated", "/estimated"));

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

    std::vector<Eigen::VectorXd> initial_states_vectors =
        urdf_kinematics->GetInitialJoints(*joint_state);
    std::vector<State> initial_states;
    for (auto state : initial_states_vectors)
    {
        initial_states.push_back(state);
    }
    tracker->initialize(initial_states);

    ROS_INFO("Running visual tracker");

    /* ------------------------------ */
    /* - Create and run tracker     - */
    /* - node                       - */
    /* ------------------------------ */
    ros::Subscriber subscriber = nh.subscribe(
        "/joint_states", 1, &dbrt::RotaryTracker::track_callback, &(*tracker));

    ros::Rate visualization_rate(24);

    while (ros::ok())
    {
        visualization_rate.sleep();
        auto current_state = tracker->current_state();

        tracker_publisher->publish(current_state, camera_data);

        ros::spinOnce();
    }

    ros::spin();

    return 0;
}
