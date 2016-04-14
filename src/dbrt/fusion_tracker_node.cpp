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
//#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Image.h>

#include <fl/util/profiling.hpp>

#include <dbot_ros/tracker_node.h>
#include <dbot/util/rigid_body_renderer.hpp>
#include <dbot/util/virtual_camera_data_provider.hpp>
#include <dbot/tracker/builder/rbc_particle_filter_tracker_builder.hpp>

#include <dbot_ros/tracker_publisher.h>
#include <dbot_ros/utils/ros_interface.hpp>
#include <dbot_ros/utils/tracking_dataset.h>
#include <dbot_ros/utils/data_set_camera_data_provider.hpp>
#include <dbot_ros/utils/ros_camera_data_provider.hpp>

#include <dbrt/robot_state.hpp>
#include <dbrt/robot_publisher.h>
#include <dbrt/util/urdf_object_loader.hpp>

#include <dbrt/robot_tracker.hpp>
#include <dbrt/rotary_tracker.hpp>
#include <dbrt/visual_tracker.hpp>
#include <dbrt/fusion_tracker.h>
#include <dbrt/fusion_tracker_node.h>

#include <dbrt/util/builder/rotary_tracker_builder.hpp>
#include <dbrt/util/builder/visual_tracker_factory.h>

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
    const int& joint_count,
    const std::vector<int>& joint_order)
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

    auto rotary_sensor_builder =
        std::make_shared<dbrt::RotarySensorBuilder<Tracker>>(sensor_parameters);

    /* ------------------------------ */
    /* - Build the tracker          - */
    /* ------------------------------ */
    auto tracker_builder = dbrt::RotaryTrackerBuilder<Tracker>(
        joint_count, joint_order, transition_builder, rotary_sensor_builder);

    return tracker_builder.build();
}

/**
 * \brief Node entry point
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_tracker");
    ros::NodeHandle nh("~");

    // parameter shorthand prefix
    std::string prefix = "fusion_tracker/";

    /* ------------------------------ */
    /* - Create the robot kinematics- */
    /* - and robot mesh model       - */
    /* ------------------------------ */

    /// \todo: the robot parameters should not be loaded inside
    /// of the URDF class, but outside, and then passed
    auto urdf_kinematics = std::make_shared<KinematicsFromURDF>();

    auto object_model = std::make_shared<dbot::ObjectModel>(
        std::make_shared<dbrt::UrdfObjectModelLoader>(urdf_kinematics), false);

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
    auto camera_data = std::make_shared<dbot::CameraData>(
        std::make_shared<dbot::RosCameraDataProvider>(nh,
                                                      camera_info_topic,
                                                      depth_image_topic,
                                                      resolution,
                                                      downsampling_factor,
                                                      2.0));

    /* ------------------------------ */
    /* - Robot renderer             - */
    /* ------------------------------ */
    auto renderer = std::make_shared<dbot::RigidBodyRenderer>(
        object_model->vertices(),
        object_model->triangle_indices(),
        camera_data->camera_matrix(),
        camera_data->resolution().height,
        camera_data->resolution().width);

    /* ------------------------------ */
    /* - Our state representation   - */
    /* ------------------------------ */
    dbrt::RobotState<>::kinematics_ = urdf_kinematics;
    dbrt::RobotState<>::kinematics_mutex_ = std::make_shared<std::mutex>();

    typedef dbrt::RobotState<> State;

    /* ------------------------------ */
    /* - Tracker publisher          - */
    /* ------------------------------ */
    auto tracker_publisher =
        std::shared_ptr<dbrt::RobotTrackerPublisher<State>>(
            new dbrt::RobotTrackerPublisher<State>(
                urdf_kinematics, renderer, "/estimated", "/estimated"));

    /* ------------------------------ */
    /* - Initialize                 - */
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

    auto joint_order = urdf_kinematics->GetJointOrder(*joint_state);

    /* ------------------------------ */
    /* - Create Tracker and         - */
    /* - tracker publisher          - */
    /* ------------------------------ */

    ROS_INFO("creating trackers ... ");
    auto gaussian_joint_robot_tracker = create_rotary_tracker(
        prefix, urdf_kinematics->num_joints(), joint_order);

    double camera_delay;
    nh.getParam(prefix + "camera_delay", camera_delay);
    dbrt::FusionTracker fusion_tracker(
        camera_data,
        gaussian_joint_robot_tracker,
        [&]()
        {
            return dbrt::create_visual_tracker(
                prefix, urdf_kinematics, object_model, camera_data);
        },
        camera_delay);

    fusion_tracker.initialize(initial_states);

    /* ------------------------------ */
    /* - Run tracker node           - */
    /* ------------------------------ */
    fusion_tracker.run();

    ros::Subscriber joint_subscriber =
        nh.subscribe("/joint_states",
                     1000,
                     &dbrt::FusionTracker::joints_obsrv_callback,
                     &fusion_tracker);

    ros::Subscriber image_subscriber =
        nh.subscribe(depth_image_topic,
                     1,
                     &dbrt::FusionTracker::image_obsrv_callback,
                     &fusion_tracker);

    ros::Rate visualization_rate(24);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while (ros::ok())
    {
        visualization_rate.sleep();
        State current_state;
        double current_state_time;
        fusion_tracker.current_state_and_time(current_state, current_state_time);


        tracker_publisher->publish_tf(current_state, ros::Time(current_state_time));
        ros::spinOnce();
    }

    ROS_INFO("Shutting down ...");
    fusion_tracker.shutdown();

    return 0;
}
