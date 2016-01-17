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
 * \file rbc_particle_filter_robot_tracker.cpp
 * \date December 2015
 * \author Jan Issac (jan.issac@gmail.com)
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#include <dbot/util/rigid_body_renderer.hpp>
#include <brt/trackers/rbc_particle_filter_robot_tracker.hpp>

namespace brt
{
RbcParticleFilterRobotTracker::RbcParticleFilterRobotTracker(
    const std::shared_ptr<Filter>& filter,
    const dbot::ObjectModel& object_model,
    const dbot::CameraData& camera_data,
    int evaluation_count)
    : RobotTracker(object_model, camera_data),
      filter_(filter),
      evaluation_count_(evaluation_count)
{
}

auto RbcParticleFilterRobotTracker::on_initialize(
    const std::vector<State>& initial_states,
    std::shared_ptr<KinematicsFromURDF>& urdf_kinematics) -> State
{
    filter_->set_particles(initial_states);
    filter_->filter(camera_data_.depth_image_vector(), zero_input());

    // determine what is the number of sampling blocks
    // eval_count / sampling blocks
    filter_->resample(evaluation_count_ / object_model_.count_parts());

    State mean = filter_->belief().mean();
    return mean;
}

auto RbcParticleFilterRobotTracker::on_track(const Obsrv& image) -> State
{
    filter_->filter(image, zero_input());

    State mean = filter_->belief().mean();
    return mean;
}

// refactor ////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//void RobotTracker::Initialize(
//    std::vector<Eigen::VectorXd> initial_samples_eigen,
//    const sensor_msgs::Image& ros_image,
//    Eigen::Matrix3d camera_matrix,
//    std::shared_ptr<KinematicsFromURDF>& urdf_kinematics)
//{
//    boost::mutex::scoped_lock lock(mutex_);

//    urdf_kinematics_ = urdf_kinematics;

//    dimension_ = urdf_kinematics->num_joints();

//    // initialize the result container for the emperical mean
//    mean_ = std::shared_ptr<State>(new State);

//    robot_renderer_ = std::shared_ptr<dbot::RigidBodyRenderer>(
//        new dbot::RigidBodyRenderer(part_vertices, part_triangle_indices));

//    // initialize process model
//    // =====================================================================================================
//    if (dimension_ != joint_sigmas.size())
//    {
//        std::cout << "the dimension of the joint sigmas is "
//                  << joint_sigmas.size() << " while the state dimension is "
//                  << dimension_ << std::endl;
//        exit(-1);
//    }
//    std::shared_ptr<ProcessModel> process(
//        new ProcessModel(delta_time, dimension_));
//    Eigen::MatrixXd joint_covariance =
//        Eigen::MatrixXd::Zero(dimension_, dimension_);
//    for (size_t i = 0; i < dimension_; i++)
//        joint_covariance(i, i) = pow(joint_sigmas[i], 2);
//    process->Parameters(damping, joint_covariance);

//}
}
