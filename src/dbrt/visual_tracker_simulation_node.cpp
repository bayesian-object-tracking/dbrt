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
 * \file visual_tracker_node.hpp
 * \date December 2015
 * \author Jan Issac (jan.issac@gmail.com)
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#include <memory>

#include <cv.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Image.h>

#include <dbot/util/rigid_body_renderer.hpp>
#include <dbot/util/virtual_camera_data_provider.hpp>
#include <dbot/tracker/builder/rbc_particle_filter_tracker_builder.hpp>

#include <dbot_ros/tracker_node.h>
#include <dbot_ros/tracker_publisher.h>
#include <dbot_ros/utils/ros_interface.hpp>
#include <dbot_ros/utils/tracking_dataset.h>
#include <dbot_ros/utils/data_set_camera_data_provider.hpp>

#include <dbrt/robot_state.hpp>
#include <dbrt/robot_tracker.hpp>
#include <dbrt/robot_tracker_publisher.h>
#include <dbrt/visual_tracker.hpp>
#include <dbrt/util/urdf_object_loader.hpp>
#include <dbrt/util/virtual_robot.h>

#include <dbrt/util/builder/visual_tracker_factory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_tracker_simulation");
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

    typedef dbrt::VisualTracker Tracker;

    // parameter shorthand prefix
    std::string pre = "particle_filter/";

    auto tracker = dbrt::create_visual_tracker(
        pre, urdf_kinematics, object_model, camera_data);

    /* ------------------------------ */
    /* - Tracker publisher          - */
    /* ------------------------------ */
    std::shared_ptr<dbot::RigidBodyRenderer> renderer(
        new dbot::RigidBodyRenderer(object_model->vertices(),
                                    object_model->triangle_indices(),
                                    camera_data->camera_matrix(),
                                    camera_data->resolution().height,
                                    camera_data->resolution().width));

    auto tracker_publisher = std::shared_ptr<dbot::TrackerPublisher<State>>(
        new dbrt::RobotTrackerPublisher<State>(
            urdf_kinematics, renderer, "/estimated"));

    /* ------------------------------ */
    /* - Create tracker node        - */
    /* ------------------------------ */
    dbot::TrackerNode<Tracker> tracker_node(tracker, tracker_publisher);

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


    std::vector<double> joints;
    nh.getParam("simulation/initial_state", joints);

    State state;
    state = Eigen::Map<Eigen::VectorXd>(joints.data(), joints.size());

    dbrt::VirtualRobot<State> robot(object_model,
                                    urdf_kinematics,
                                    simulation_renderer,
                                    simulation_camera_data,
                                    3,
                                    3,
                                    state);

    /* ------------------------------ */
    /* - Initialize                 - */
    /* ------------------------------ */
    tracker->initialize({robot.state()});
    robot.run();

    /* ------------------------------ */
    /* - Create and run tracker     - */
    /* - node                       - */
    /* ------------------------------ */
    while (ros::ok())
    {
        tracker_node.tracking_callback(robot.observation());
        ros::spinOnce();
    }

    return 0;
}
