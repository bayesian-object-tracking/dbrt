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
#include <dbrt/rbc_particle_filter_robot_tracker.hpp>
#include <dbrt/util/urdf_object_loader.hpp>
#include <dbrt/util/builder/rbc_particle_filter_robot_tracker_builder.hpp>
#include <dbrt/util/virtual_robot.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rbc_particle_filter_robot_tracker_simulation");
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

    /* ------------------------------ */
    /* - Simulator                  - */
    /* ------------------------------ */
    std::shared_ptr<dbot::RigidBodyRenderer> renderer(
        new dbot::RigidBodyRenderer(object_model->vertices(),
                                    object_model->triangle_indices(),
                                    camera_data->camera_matrix(),
                                    camera_data->resolution().height,
                                    camera_data->resolution().width));

    dbrt::VirtualRobot<State> robot(
        object_model, urdf_kinematics, renderer, camera_data);

    /* ------------------------------ */
    /* - Create and run tracker     - */
    /* - node                       - */
    /* ------------------------------ */
    std::vector<double> joints;
    nh.getParam("joints", joints);

    State state;
    state = Eigen::Map<Eigen::VectorXd>(joints.data(), joints.size());

    while (ros::ok())
    {
        robot.publish(state);
        usleep(1000);
        ros::spinOnce();
    }

    //    ROS_INFO("Start playback");
    //    for (size_t i = 1; i < data_set->Size() && ros::ok(); i++)
    //    {
    //        auto image  = *data_set->GetImage(i);
    //        image.header.frame_id = "/estimate/XTION";
    //        image.header.stamp = ros::Time::now();
    //        tracker_node.tracking_callback(image);

    //        image_publisher.publish(image);
    //        pcl::PointCloud<pcl::PointXYZ> point_cloud =
    //        *(data_set->GetPointCloud(i));

    //        point_cloud.header.frame_id = "/estimate/XTION";
    //        point_cloud.header.stamp = ros::Time::now();
    //        cloud_publisher.publish(point_cloud);

    //        usleep(1e6);
    //    }

    return 0;
}
