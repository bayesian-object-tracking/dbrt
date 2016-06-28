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

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <fl/util/profiling.hpp>

#include <dbot/rigid_body_renderer.hpp>
#include <dbot/virtual_camera_data_provider.hpp>
#include <dbot/builder/rbc_particle_filter_tracker_builder.hpp>

#include <dbot_ros/util/ros_interface.hpp>
#include <dbot_ros/util/tracking_dataset.h>
#include <dbot_ros/util/data_set_camera_data_provider.hpp>
#include <dbot_ros/util/ros_camera_data_provider.hpp>

#include <dbrt/robot_state.hpp>
#include <dbrt/robot_publisher.h>
#include <dbrt/urdf_object_loader.h>

#include <dbrt/tracker/robot_tracker.h>
#include <dbrt/tracker/rotary_tracker.h>
#include <dbrt/tracker/visual_tracker.h>
#include <dbrt/tracker/fusion_tracker.h>

#include <dbrt/tracker/visual_tracker_factory.h>
#include <dbrt/tracker/rotary_tracker_factory.h>

/**
 * \brief Node entry point
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_tracker");
    ros::NodeHandle nh("~");

    /* ------------------------------ */
    /* - Setup camera data          - */
    /* ------------------------------ */
    dbot::CameraData::Resolution resolution;
    resolution.width = ri::read<int>("resolution/width", nh);
    resolution.height = ri::read<int>("resolution/height", nh);

    auto camera_data = std::make_shared<dbot::CameraData>(
        std::make_shared<dbot::RosCameraDataProvider>(
            nh,
            ri::read<std::string>("camera_info_topic", nh),
            ri::read<std::string>("depth_image_topic", nh),
            resolution,
            ri::read<int>("downsampling_factor", nh),
            2.0));

    // parameter shorthand prefix
    std::string prefix = "fusion_tracker/";

    /* ------------------------------ */
    /* - Create the robot kinematics- */
    /* - and robot mesh model       - */
    /* ------------------------------ */
    std::string prefixed_frame_id = camera_data->frame_id();
    std::size_t slash_index = prefixed_frame_id.find_last_of("/");
    std::string frame_id = prefixed_frame_id.substr(slash_index + 1);

    std::shared_ptr<KinematicsFromURDF> urdf_kinematics(new KinematicsFromURDF(
        ri::read<std::string>("robot_description", ros::NodeHandle()),
        ri::read<std::string>("robot_description_package_path", nh),
        ri::read<std::string>("rendering_root_left", nh),
        ri::read<std::string>("rendering_root_right", nh),
        frame_id));

    auto object_model = std::make_shared<dbot::ObjectModel>(
        std::make_shared<dbrt::UrdfObjectModelLoader>(urdf_kinematics), false);

    /* ------------------------------ */
    /* - Our state representation   - */
    /* ------------------------------ */
    typedef dbrt::RobotState<> State;
    dbrt::RobotState<>::kinematics_ = urdf_kinematics;
    dbrt::RobotState<>::kinematics_mutex_ = std::make_shared<std::mutex>();

    /* ------------------------------ */
    /* - Tracker publisher          - */
    /* ------------------------------ */
    auto tracker_publisher =
        std::make_shared<dbrt::RobotTrackerPublisher<State>>(
            urdf_kinematics,
            "/estimated",
            "/estimated",
            ri::read<std::string>("tf_connecting_frame", nh));

    /* ------------------------------ */
    /* - Get initial state          - */
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
    for (auto state : initial_states_vectors) initial_states.push_back(state);

    /* ------------------------------ */
    /* - Create Tracker             - */
    /* ------------------------------ */

    ROS_INFO("creating trackers ... ");
    auto gaussian_joint_robot_tracker = dbrt::create_rotary_tracker(
        prefix,
        urdf_kinematics->num_joints(),
        urdf_kinematics->GetJointOrder(*joint_state));

    auto camera_delay = ri::read<double>(prefix + "camera_delay", nh);
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
        nh.subscribe(ri::read<std::string>("depth_image_topic", nh),
                     1,
                     &dbrt::FusionTracker::image_obsrv_callback,
                     &fusion_tracker);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate visualization_rate(24);
    while (ros::ok())
    {
        visualization_rate.sleep();

        State current_state;
        double current_time;
        dbrt::JointsObsrv current_angle_measurement;
        fusion_tracker.current_things(
            current_state, current_time, current_angle_measurement);

        if (current_angle_measurement.size() > 0)
        {
            tracker_publisher->publish_tf(current_state,
                                          current_angle_measurement,
                                          ros::Time(current_time));
            // tracker_publisher->publish_tf(current_state,
            //                               ros::Time(current_time));
        }

        ros::spinOnce();
    }

    ROS_INFO("Shutting down ...");
    fusion_tracker.shutdown();

    return 0;
}
