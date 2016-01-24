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

#include <brt/utils/urdf_object_loader.hpp>

#include <brt/states/robot_state.hpp>
#include <brt/trackers/robot_tracker.hpp>
#include <dbot/util/rigid_body_renderer.hpp>
#include <dbot/tracker/builder/rbc_particle_filter_tracker_builder.hpp>
#include <brt/robot_tracker_publisher.h>
#include <brt/trackers/rbc_particle_filter_robot_tracker.hpp>
#include <brt/trackers/builder/rbc_particle_filter_robot_tracker_builder.hpp>

// void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
//{
//    boost::mutex::scoped_lock lock(joint_state_mutex_);
//    joint_state_ = *msg;

//    if (first_time_) first_time_ = false;
//}

//    boost::mutex mutex_;
//    ros::NodeHandle nh_;
//    ros::NodeHandle priv_nh_;

//    sensor_msgs::Image ros_image_;

//    sensor_msgs::JointState joint_state_;
//    sensor_msgs::JointState joint_state_copy_;
//    boost::mutex joint_state_mutex_;

//    ros::Subscriber subscriber_;

//    bool first_time_;
//    bool has_image_;
//    bool has_joints_;

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
        new brt::UrdfObjectModelLoader(urdf_kinematics));

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
    typedef brt::RobotState<> State;

    // very important
    State::kinematics_ = urdf_kinematics;

    typedef brt::RbcParticleFilterRobotTracker Tracker;
    typedef brt::RbcParticleFilterRobotTrackerBuilder<Tracker> TrackerBuilder;
    typedef TrackerBuilder::StateTransitionBuilder StateTransitionBuilder;
    typedef TrackerBuilder::ObservationModelBuilder ObservationModelBuilder;

    // parameter shorthand prefix
    std::string pre = "particle_filter/";

    /* ------------------------------ */
    /* - State transition function  - */
    /* ------------------------------ */
    // We will use a linear observation model built by the object transition
    // model builder. The linear model will generate a random walk.
    brt::RobotJointTransitionModelBuilder<State>::Parameters params_state;
    // linear state transition parameters
    nh.getParam(pre + "joint_transition/joint_sigma", params_state.joint_sigma);
    nh.getParam(pre + "joint_transition/velocity_factor",
                params_state.velocity_factor);
    params_state.joint_count = urdf_kinematics->num_joints();

    auto state_trans_builder = std::shared_ptr<StateTransitionBuilder>(
        new brt::RobotJointTransitionModelBuilder<State>(params_state));

    /* ------------------------------ */
    /* - Observation model          - */
    /* ------------------------------ */
    dbot::RbObservationModelBuilder<State>::Parameters params_obsrv;
    nh.getParam(pre + "use_gpu", params_obsrv.use_gpu);

    if (params_obsrv.use_gpu)
    {
        nh.getParam(pre + "gpu/sample_count", params_obsrv.sample_count);
    }
    else
    {
        nh.getParam(pre + "cpu/sample_count", params_obsrv.sample_count);
    }

    nh.getParam(pre + "observation/occlusion/p_occluded_visible",
                params_obsrv.occlusion.p_occluded_visible);
    nh.getParam(pre + "observation/occlusion/p_occluded_occluded",
                params_obsrv.occlusion.p_occluded_occluded);
    nh.getParam(pre + "observation/occlusion/initial_occlusion_prob",
                params_obsrv.occlusion.initial_occlusion_prob);

    nh.getParam(pre + "observation/kinect/tail_weight",
                params_obsrv.kinect.tail_weight);
    nh.getParam(pre + "observation/kinect/model_sigma",
                params_obsrv.kinect.model_sigma);
    nh.getParam(pre + "observation/kinect/sigma_factor",
                params_obsrv.kinect.sigma_factor);
    params_obsrv.delta_time = 1. / 30.;

    // gpu only parameters
    nh.getParam(pre + "gpu/use_custom_shaders",
                params_obsrv.use_custom_shaders);
    nh.getParam(pre + "gpu/vertex_shader_file",
                params_obsrv.vertex_shader_file);
    nh.getParam(pre + "gpu/fragment_shader_file",
                params_obsrv.fragment_shader_file);
    nh.getParam(pre + "gpu/geometry_shader_file",
                params_obsrv.geometry_shader_file);

    auto obsrv_model_builder = std::shared_ptr<ObservationModelBuilder>(
        new dbot::RbObservationModelBuilder<State>(
            object_model, camera_data, params_obsrv));

    /* ------------------------------ */
    /* - Create Filter & Tracker    - */
    /* ------------------------------ */
    TrackerBuilder::Parameters params_tracker;
    params_tracker.evaluation_count = params_obsrv.sample_count;
    nh.getParam(pre + "moving_average_update_rate",
                params_tracker.moving_average_update_rate);
    nh.getParam(pre + "max_kl_divergence", params_tracker.max_kl_divergence);

    auto tracker_builder =
        brt::RbcParticleFilterRobotTrackerBuilder<Tracker>(urdf_kinematics,
                                                           state_trans_builder,
                                                           obsrv_model_builder,
                                                           object_model,
                                                           camera_data,
                                                           params_tracker);

    auto tracker = tracker_builder.build();

    /* ------------------------------ */
    /* - Tracker publisher          - */
    /* ------------------------------ */
    auto tracker_publisher = std::shared_ptr<dbot::TrackerPublisher<Tracker>>(
        new brt::RobotTrackerPublisher<Tracker>(
             urdf_kinematics, obsrv_model_builder->create_renderer()));

    /* ------------------------------ */
    /* - Create tracker node        - */
    /* ------------------------------ */
    dbot::TrackerNode<Tracker> tracker_node(tracker, tracker_publisher);

    /* ------------------------------ */
    /* - Initialize using joint msg - */
    /* ------------------------------ */
    //    ros::Subscriber joint_states_sub =
    //    nh.subscribe<sensor_msgs::JointState>(
    //        "/joint_states", 1, &jointStateCallback, this);

    //    while (!has_joints)
    //    {
    //        ROS_INFO("Waiting for joint angles.");
    //        ros::spinOnce();
    //        ros::Rate(30).sleep();
    //    }

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
    tracker->initialize(initial_states, urdf_kinematics);

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
