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

#include <ros/ros.h>
//#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Image.h>

#include <dbot/util/rigid_body_renderer.hpp>
#include <dbot/tracker/builder/rbc_particle_filter_tracker_builder.hpp>

#include <dbot_ros/tracker_node.h>
#include <dbot_ros/tracker_publisher.h>
#include <dbot_ros/utils/ros_interface.hpp>
#include <dbot_ros/utils/tracking_dataset.h>
#include <dbot_ros/utils/data_set_camera_data_provider.hpp>

#include <dbrt/robot_state.hpp>
#include <dbrt/robot_tracker.hpp>
#include <dbrt/robot_publisher.h>
#include <dbrt/visual_tracker.hpp>
#include <dbrt/util/urdf_object_loader.hpp>
#include <dbrt/util/builder/visual_tracker_factory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_tracker_offline");
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
    /* - Data set provider          - */
    /* ------------------------------ */
    std::string source;
    nh.getParam("source", source);

    ROS_INFO_STREAM("Loading data from " << source);
    auto data_set = std::make_shared<TrackingDataset>(source);
    ROS_INFO("Data set loaded.");

    /* ------------------------------ */
    /* - Setup camera data          - */
    /* ------------------------------ */
    int downsampling_factor;
    nh.getParam("downsampling_factor", downsampling_factor);

    auto camera_data_provider = std::shared_ptr<dbot::CameraDataProvider>(
        new dbot::DataSetCameraDataProvider(data_set, downsampling_factor));

    // Create camera data from the DataSetCameraDataProvider which takes the
    // data from the loaded tracking data set object
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

    // parameter shorthand prefix
    std::string pre = "particle_filter/";

    auto tracker = dbrt::create_visual_tracker(
        pre, urdf_kinematics, object_model, camera_data);

    auto data_camera_data_provider = std::shared_ptr<dbot::CameraDataProvider>(
        new dbot::DataSetCameraDataProvider(data_set, 1));

    // Create camera data from the DataSetCameraDataProvider which takes the
    // data from the loaded tracking data set object
    auto data_camera_data =
        std::make_shared<dbot::CameraData>(data_camera_data_provider);

    std::shared_ptr<dbot::RigidBodyRenderer> data_renderer(
        new dbot::RigidBodyRenderer(object_model->vertices(),
                                    object_model->triangle_indices(),
                                    data_camera_data->camera_matrix(),
                                    data_camera_data->resolution().height,
                                    data_camera_data->resolution().width));

    /* ------------------------------ */
    /* - Tracker publisher          - */
    /* ------------------------------ */
    auto tracker_publisher = std::shared_ptr<dbot::TrackerPublisher<State>>(
        new dbrt::RobotTrackerPublisher<State>(
            urdf_kinematics, data_renderer, "/estimated", "/estimated"));

    auto data_tracker_publisher =
        std::make_shared<dbrt::RobotTrackerPublisher<State>>(
            urdf_kinematics, data_renderer, "/sensors", "");

    /* ------------------------------ */
    /* - Create tracker node        - */
    /* ------------------------------ */
    dbot::TrackerNode<dbrt::VisualTracker> tracker_node(tracker,
                                                        camera_data,
                                                        tracker_publisher);

    /* ------------------------------ */
    /* - Initialize                 - */
    /* ------------------------------ */
    std::vector<Eigen::VectorXd> initial_states_vectors(
        1, data_set->GetGroundTruth(0));
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

    ROS_INFO("Start playback");
    for (size_t i = 1; i < data_set->Size() && ros::ok(); i++)
    {
        auto image = *data_set->GetImage(i);
        tracker_node.tracking_callback(image);
        data_tracker_publisher->publish(
            tracker_node.current_state(), image, data_camera_data);

        ros::spinOnce();
    }

    return 0;
}
