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
 * \file rbc_particle_filter_robot_tracker_node.hpp
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
#include <dbrt/robot_tracker_publisher.h>
#include <dbrt/rbc_particle_filter_robot_tracker.hpp>

#include <dbrt/util/builder/ros_rbc_particle_filter_robot_tracker_factory.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rbc_particle_filter_robot_tracker");
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

    typedef dbrt::RbcParticleFilterRobotTracker Tracker;

    // parameter shorthand prefix
    std::string pre = "particle_filter/";

    auto tracker = dbrt::create_rbc_particle_filter_robot_tracker(
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
            urdf_kinematics,
            renderer,
            "/estimated"));

    /* ------------------------------ */
    /* - Create tracker node        - */
    /* ------------------------------ */
    dbot::TrackerNode<Tracker> tracker_node(tracker, tracker_publisher);

    /* ------------------------------ */
    /* - Initialize using joint msg - */
    /* ------------------------------ */
    sensor_msgs::JointState::ConstPtr joint_state;

    while (!joint_state)
    {
        joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>(
            "/joint_states", nh, ros::Duration(1.));
        ROS_INFO("Waiting for initial joint state");
    }

    std::vector<Eigen::VectorXd> initial_states_vectors =
        urdf_kinematics->GetInitialJoints(*joint_state);
    std::vector<State> initial_states;
    for (auto state : initial_states_vectors)
    {
        initial_states.push_back(state);
    }
    tracker->initialize(initial_states);

    /* ------------------------------ */
    /* - Create and run tracker     - */
    /* - node                       - */
    /* ------------------------------ */
    ros::Subscriber subscriber =
        nh.subscribe(depth_image_topic,
                     1,
                     &dbot::TrackerNode<Tracker>::tracking_callback,
                     &tracker_node);

    ros::spin();

    return 0;
}
