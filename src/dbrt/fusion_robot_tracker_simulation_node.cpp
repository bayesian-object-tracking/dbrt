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
#include <dbrt/fusion_robot_tracker.h>
#include <dbrt/fusion_tracker_node.h>
#include <dbrt/robot_tracker_publisher.h>
#include <dbrt/util/urdf_object_loader.hpp>
#include <dbrt/util/virtual_robot.h>
#include <dbrt/util/builder/fusion_robot_tracker_builder.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_robot_tracker_simulation");
    ros::NodeHandle nh("~");

    /* ------------------------------ */
    /* - Create the robot model     - */
    /* ------------------------------ */
    // initialize the kinematics
    std::shared_ptr<KinematicsFromURDF> urdf_kinematics(
        new KinematicsFromURDF());

    auto object_model_loader = std::shared_ptr<dbot::ObjectModelLoader>(
        new dbrt::UrdfObjectModelLoader(urdf_kinematics));

    auto object_model =
        std::make_shared<dbot::ObjectModel>(object_model_loader, false);

    /* ------------------------------ */
    /* - Setup camera data          - */
    /* ------------------------------ */
    int downsampling_factor;
    nh.getParam("downsampling_factor", downsampling_factor);

    auto camera_data_provider = std::shared_ptr<dbot::CameraDataProvider>(
        new dbot::VirtualCameraDataProvider(downsampling_factor, "/XTION"));

    auto camera_data = std::make_shared<dbot::CameraData>(camera_data_provider);

    /* ------------------------------ */
    /* - Few types we will be using - */
    /* ------------------------------ */
    dbrt::RobotState<>::kinematics_ = urdf_kinematics;
    typedef dbrt::RobotState<> State;

    typedef dbrt::FusionRobotTracker Tracker;
    typedef dbrt::FusionRobotTrackerBuilder<Tracker> TrackerBuilder;
    typedef TrackerBuilder::StateTransitionBuilder StateTransitionBuilder;
    typedef TrackerBuilder::ObservationModelBuilder ObservationModelBuilder;

    // parameter shorthand prefix
    std::string pre = "fusion_tracker/";

    /* ------------------------------ */
    /* - State transition function  - */
    /* ------------------------------ */
    // We will use a linear observation model built by the object transition
    // model builder. The linear model will generate a random walk.
    dbrt::RobotJointTransitionModelBuilder::Parameters params_state;

    // linear state transition parameters
    nh.getParam(pre + "joint_transition/joint_sigmas",
                params_state.joint_sigmas);
    params_state.joint_count = urdf_kinematics->num_joints();

    auto state_trans_builder =
        std::make_shared<StateTransitionBuilder>((params_state));

    /* ------------------------------ */
    /* - Observation model          - */
    /* ------------------------------ */
    dbrt::RobotJointObservationModelBuilder::Parameters
        params_joint_obsrv;

    nh.getParam(pre + "joint_observation/joint_sigmas",
                params_joint_obsrv.joint_sigmas);
    params_joint_obsrv.joint_count = urdf_kinematics->num_joints();

    auto joint_obsrv_model_builder = std::shared_ptr<ObservationModelBuilder>(
        new ObservationModelBuilder(params_joint_obsrv));

    /* ------------------------------ */
    /* - Robot renderer             - */
    /* ------------------------------ */
    std::shared_ptr<dbot::RigidBodyRenderer> renderer(
        new dbot::RigidBodyRenderer(object_model->vertices(),
                                    object_model->triangle_indices(),
                                    camera_data->camera_matrix(),
                                    camera_data->resolution().height,
                                    camera_data->resolution().width));

    /* ------------------------------ */
    /* - Create Filter & Tracker    - */
    /* ------------------------------ */
    auto tracker_builder =
        dbrt::FusionRobotTrackerBuilder<Tracker>(urdf_kinematics,
                                                 state_trans_builder,
                                                 joint_obsrv_model_builder,
                                                 object_model,
                                                 camera_data);
    auto tracker = tracker_builder.build();

    /* ------------------------------ */
    /* - Tracker publisher          - */
    /* ------------------------------ */
    auto tracker_publisher = std::shared_ptr<dbot::TrackerPublisher<State>>(
        new dbrt::RobotTrackerPublisher<State>(
            urdf_kinematics, renderer, "/estimated"));

    /* ------------------------------ */
    /* - Create tracker node        - */
    /* ------------------------------ */
    dbrt::FusionTrackerNode<Tracker> tracker_node(tracker, tracker_publisher);

    /* ------------------------------ */
    /* - Simulation                 - */
    /* ------------------------------ */
    auto simulation_camera_data_provider =
        std::shared_ptr<dbot::CameraDataProvider>(
            new dbot::VirtualCameraDataProvider(1, "/XTION"));
    auto simulation_camera_data =
        std::make_shared<dbot::CameraData>(simulation_camera_data_provider);

    std::shared_ptr<dbot::RigidBodyRenderer> simulation_renderer(
        new dbot::RigidBodyRenderer(
            object_model->vertices(),
            object_model->triangle_indices(),
            simulation_camera_data->camera_matrix(),
            simulation_camera_data->resolution().height,
            simulation_camera_data->resolution().width));

    dbrt::VirtualRobot<State> robot(object_model,
                                    urdf_kinematics,
                                    simulation_renderer,
                                    simulation_camera_data);

    /* ------------------------------ */
    /* - Initialize                 - */
    /* ------------------------------ */
    std::vector<double> joints;
    nh.getParam("simulation/initial_state", joints);

    State state;
    state = Eigen::Map<Eigen::VectorXd>(joints.data(), joints.size());
    State init_state = robot.animate(state);
    tracker->initialize({init_state}, robot.observation_vector());

    /* ------------------------------ */
    /* - Create and run tracker     - */
    /* - node                       - */
    /* ------------------------------ */
    while (ros::ok())
    {
        auto new_state = robot.animate(state);
        robot.publish(new_state);

        tracker_node.tracking_callback(robot.observation(),
                                       new_state);
        ros::spinOnce();
    }

    return 0;
}
