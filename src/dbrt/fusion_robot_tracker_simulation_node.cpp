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
 * \file fusion_robot_tracker_simulation_node.hpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <memory>

#include <cv.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Image.h>

#include <fl/util/profiling.hpp>

#include <dbot/util/rigid_body_renderer.hpp>
#include <dbot/util/virtual_camera_data_provider.hpp>
#include <dbot/tracker/builder/rbc_particle_filter_tracker_builder.hpp>

#include <dbot_ros/tracker_publisher.h>
#include <dbot_ros/utils/ros_interface.hpp>
#include <dbot_ros/utils/tracking_dataset.h>
#include <dbot_ros/utils/data_set_camera_data_provider.hpp>

#include <dbrt/robot_state.hpp>
#include <dbrt/robot_tracker.hpp>
#include <dbrt/fusion_tracker_node.h>
#include <dbrt/robot_tracker_publisher.h>
#include <dbrt/util/urdf_object_loader.hpp>
#include <dbrt/util/virtual_robot.h>
#include <dbrt/gaussian_joint_filter_robot_tracker.hpp>
#include <dbrt/util/builder/gaussian_joint_filter_robot_tracker_builder.hpp>
#include <dbrt/gaussian_joint_filter_robot_tracker.hpp>

/**
 * \brief Create a gaussian filter tracking the robot joints based on joint
 *     measurements
 * \param prefix
 *     parameter prefix, e.g. fusion_robot_tracker
 * \param urdf_kinematics
 *     URDF robot kinematics
 */
std::shared_ptr<dbrt::GaussianJointFilterRobotTracker>
create_joint_robot_tracker(
    const std::string& prefix,
    const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics)
{
    ros::NodeHandle nh("~");

    typedef dbrt::GaussianJointFilterRobotTracker Tracker;

    /* ------------------------------ */
    /* - State transition function  - */
    /* ------------------------------ */
    dbrt::RobotJointTransitionModelBuilder<Tracker>::Parameters params_state;

    // linear state transition parameters
    nh.getParam(prefix + "joint_transition/joint_sigmas",
                params_state.joint_sigmas);
    params_state.joint_count = urdf_kinematics->num_joints();

    auto state_trans_builder =
        std::make_shared<dbrt::RobotJointTransitionModelBuilder<Tracker>>(
            (params_state));

    /* ------------------------------ */
    /* - Observation model          - */
    /* ------------------------------ */
    dbrt::RobotJointObservationModelBuilder<Tracker>::Parameters
        params_joint_obsrv;

    nh.getParam(prefix + "joint_observation/joint_sigmas",
                params_joint_obsrv.joint_sigmas);
    params_joint_obsrv.joint_count = urdf_kinematics->num_joints();

    auto joint_obsrv_model_builder =
        std::make_shared<dbrt::RobotJointObservationModelBuilder<Tracker>>(
            params_joint_obsrv);

    /* ------------------------------ */
    /* - Build the tracker          - */
    /* ------------------------------ */
    auto tracker_builder =
        dbrt::GaussianJointFilterRobotTrackerBuilder<Tracker>(
            urdf_kinematics, state_trans_builder, joint_obsrv_model_builder);

    return tracker_builder.build();
}

/**
 * \brief Node entry point
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_robot_tracker_simulation");
    ros::NodeHandle nh("~");

    // parameter shorthand prefix
    std::string pre = "fusion_tracker/";

    /* ------------------------------ */
    /* - Create the robot kinematics- */
    /* - and robot mesh model       - */
    /* ------------------------------ */
    auto urdf_kinematics = std::make_shared<KinematicsFromURDF>();

    auto object_model = std::make_shared<dbot::ObjectModel>(
        std::make_shared<dbrt::UrdfObjectModelLoader>(urdf_kinematics), false);

    /* ------------------------------ */
    /* - Setup camera data          - */
    /* ------------------------------ */
    int downsampling_factor;
    nh.getParam("downsampling_factor", downsampling_factor);
    auto camera_data = std::make_shared<dbot::CameraData>(
        std::make_shared<dbot::VirtualCameraDataProvider>(downsampling_factor,
                                                          "/XTION"));

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
    typedef dbrt::RobotState<> State;

    /* ------------------------------ */
    /* - Create Tracker and         - */
    /* - tracker publisher          - */
    /* ------------------------------ */
    auto tracker = create_joint_robot_tracker(pre, urdf_kinematics);

    auto tracker_publisher = std::shared_ptr<dbot::TrackerPublisher<State>>(
        new dbrt::RobotTrackerPublisher<State>(
            urdf_kinematics, renderer, "/estimated"));

    /* ------------------------------ */
    /* - Setup Simulation           - */
    /* ------------------------------ */
    auto simulation_camera_data = std::make_shared<dbot::CameraData>(
        std::make_shared<dbot::VirtualCameraDataProvider>(1, "/XTION"));

    auto simulation_renderer = std::make_shared<dbot::RigidBodyRenderer>(
            object_model->vertices(),
            object_model->triangle_indices(),
            simulation_camera_data->camera_matrix(),
            simulation_camera_data->resolution().height,
            simulation_camera_data->resolution().width);

    auto robot = dbrt::VirtualRobot<State>(object_model,
                                    urdf_kinematics,
                                    simulation_renderer,
                                    simulation_camera_data);

    /* ------------------------------ */
    /* - Initialize from config     - */
    /* ------------------------------ */
    std::vector<double> joints;
    nh.getParam("simulation/initial_state", joints);

    State state;
    state = Eigen::Map<Eigen::VectorXd>(joints.data(), joints.size());
    State init_state = robot.animate(state);
    tracker->initialize({init_state}, robot.observation_vector());

    /* ------------------------------ */
    /* - Run tracker node           - */
    /* ------------------------------ */
    while (ros::ok())
    {
        auto new_state = robot.animate(state);
        robot.publish(new_state);

        auto current_state = tracker->track(new_state);
        tracker_publisher->publish(
            current_state, robot.observation(), camera_data);
        ros::spinOnce();
    }

    return 0;
}
