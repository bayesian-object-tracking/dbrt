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
    transition_parameters.joint_sigmas = ri::read<std::vector<double>> (
                                 prefix + "joint_transition/joint_sigmas", nh);
    transition_parameters.bias_sigmas = ri::read<std::vector<double>> (
                                 prefix + "joint_transition/bias_sigmas", nh);
    transition_parameters.bias_factors = ri::read<std::vector<double>> (
                                 prefix + "joint_transition/bias_factors", nh);
    transition_parameters.joint_count = joint_count;

    auto transition_builder =
        std::make_shared<dbrt::FactorizedTransitionBuilder<Tracker>>(
            (transition_parameters));

    /* ------------------------------ */
    /* - Observation model          - */
    /* ------------------------------ */
    dbrt::RotarySensorBuilder<Tracker>::Parameters sensor_parameters;

    sensor_parameters.joint_sigmas = ri::read<std::vector<double>>(
                                 prefix + "joint_observation/joint_sigmas", nh);
    sensor_parameters.joint_count = joint_count;

    PV(joint_count);

    auto rotary_sensor_builder =
        std::make_shared<dbrt::RotarySensorBuilder<Tracker>>(sensor_parameters);

    /* ------------------------------ */
    /* - Build the tracker          - */
    /* ------------------------------ */
    auto tracker_builder = dbrt::RotaryTrackerBuilder<Tracker>(
        joint_count, joint_order, transition_builder, rotary_sensor_builder);

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
    /* - Setup camera data          - */
    /* ------------------------------ */
    auto camera_info_topic = ri::read<std::string>("camera_info_topic", nh);
    auto depth_image_topic = ri::read<std::string>("depth_image_topic", nh);
    auto downsampling_factor = ri::read<int>("downsampling_factor", nh);
    dbot::CameraData::Resolution resolution;
    resolution.width = ri::read<int>("resolution/width", nh);
    resolution.height = ri::read<int>("resolution/height", nh);

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
    /* - Create the robot model     - */
    /* ------------------------------ */
    // initialize the kinematics
    auto robot_description =
            ri::read<std::string>("robot_description",
                                            ros::NodeHandle());
    auto robot_description_package_path =
            ri::read<std::string>("robot_description_package_path", nh);
    auto rendering_root_left =
            ri::read<std::string>("rendering_root_left", nh);
    auto rendering_root_right =
            ri::read<std::string>("rendering_root_right", nh);

    std::shared_ptr<KinematicsFromURDF> urdf_kinematics(
                new KinematicsFromURDF(robot_description,
                                       robot_description_package_path,
                                       rendering_root_left,
                                       rendering_root_right,
                                       "NO_CAMERA_FRAME"));

    auto object_model_loader = std::shared_ptr<dbot::ObjectModelLoader>(
        new dbrt::UrdfObjectModelLoader(urdf_kinematics));

    // Load the model usign the URDF loader
    auto object_model =
        std::make_shared<dbot::ObjectModel>(object_model_loader, false);



    /* ------------------------------ */
    /* - Few types we will be using - */
    /* ------------------------------ */
    dbrt::RobotState<>::kinematics_ = urdf_kinematics;
    dbrt::RobotState<>::kinematics_mutex_ = std::make_shared<std::mutex>();
    typedef dbrt::RobotState<> State;

    typedef dbrt::RotaryTracker Tracker;

    // parameter shorthand prefix
    std::string pre = "rotary_tracker/";

    //    urdf_kinematics->set_joint_angles(Eigen::VectorXd::Zero(urdf_kinematics->num_joints()));
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

    auto joint_order = urdf_kinematics->GetJointOrder(*joint_state);

    auto tracker =
        create_rotary_tracker(pre, urdf_kinematics->num_joints(), joint_order);

    /* ------------------------------ */
    /* - Tracker publisher          - */
    /* ------------------------------ */

    std::shared_ptr<dbot::RigidBodyRenderer> renderer(
        new dbot::RigidBodyRenderer(object_model->vertices(),
                                    object_model->triangle_indices(),
                                    camera_data->camera_matrix(),
                                    camera_data->resolution().height,
                                    camera_data->resolution().width));

    auto tf_connecting_frame =
            ri::read<std::string>("tf_connecting_frame", nh);

    auto tracker_publisher =
        std::shared_ptr<dbrt::RobotTrackerPublisher<State>>(
            new dbrt::RobotTrackerPublisher<State>(
                urdf_kinematics, renderer, "/estimated", "/estimated", tf_connecting_frame));

    std::vector<Eigen::VectorXd> initial_states_vectors =
        urdf_kinematics->GetInitialJoints(*joint_state);
    std::vector<State> initial_states;
    for (auto state : initial_states_vectors)
    {
        initial_states.push_back(state);
    }
    tracker->initialize(initial_states);

    ROS_INFO("Running rotary tracker");

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

        /// \todo: THIS IS A HACK!! WE SHOULD PASS THE PROPER TIME WHICH
        /// CORRESPONDS TO THE MEASUREMENT
        std::cout << "PUBLISHING ESTIMATED JONT ANGLES AND TF WITH"
                     "NOW() TIMESTAMP. THIS HAS TO BE FIXED!!!!" << std::endl;
        ros::Time time = ros::Time::now();
        tracker_publisher->publish_joint_state(current_state, time);
        tracker_publisher->publish_tf(current_state, time);

        ros::spinOnce();
    }

    ros::spin();

    return 0;
}
