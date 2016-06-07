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
#include <dbot/common/rigid_body_renderer.hpp>
#include <dbrt/robot_publisher.h>
#include <dbrt/visual_tracker.hpp>

#include <dbrt/util/builder/visual_tracker_factory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_tracker");
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
        ri::read<std::string>("robot_description", ros::NodeHandle());
    auto robot_description_package_path =
        ri::read<std::string>("robot_description_package_path", nh);
    auto rendering_root_left = ri::read<std::string>("rendering_root_left", nh);
    auto rendering_root_right =
        ri::read<std::string>("rendering_root_right", nh);

    std::string prefixed_frame_id = camera_data->frame_id();
    std::size_t slash_index = prefixed_frame_id.find_last_of("/");
    std::string frame_id = prefixed_frame_id.substr(slash_index + 1);

    std::shared_ptr<KinematicsFromURDF> urdf_kinematics(
        new KinematicsFromURDF(robot_description,
                               robot_description_package_path,
                               rendering_root_left,
                               rendering_root_right,
                               frame_id));

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

    /* ------------------------------ */
    /* - Tracker publisher          - */
    /* ------------------------------ */

    std::shared_ptr<dbot::RigidBodyRenderer> renderer(
        new dbot::RigidBodyRenderer(object_model->vertices(),
                                    object_model->triangle_indices(),
                                    camera_data->camera_matrix(),
                                    camera_data->resolution().height,
                                    camera_data->resolution().width));

    auto tf_connecting_frame = ri::read<std::string>("tf_connecting_frame", nh);

    auto tracker_publisher =
        std::shared_ptr<dbrt::RobotTrackerPublisher<State>>(
            new dbrt::RobotTrackerPublisher<State>(urdf_kinematics,
                                                   renderer,
                                                   "/estimated",
                                                   "/estimated",
                                                   tf_connecting_frame));

    /* ------------------------------ */
    /* - Create tracker             - */
    /* ------------------------------ */
    // parameter shorthand prefix
    std::string pre = "particle_filter/";
    auto tracker = dbrt::create_visual_tracker(
        pre, urdf_kinematics, object_model, camera_data);

    // dbot::TrackerNode<dbrt::VisualTracker> tracker_node(tracker,
    //                                                     camera_data,
    //                                                     tracker_publisher);

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
    // ros::Subscriber subscriber =
    //     nh.subscribe(depth_image_topic,
    //                  1,
    //                  &dbot::TrackerNode<dbrt::VisualTracker>::tracking_callback,
    //                  &tracker_node);

    ros::spin();

    return 0;
}
