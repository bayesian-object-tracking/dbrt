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

/**
 * \file kinematics_from_urdf.cpp
 * \date 2014, 2016
 * \author Jeannette Bohg (jbohg@tuebingen.mpg.de)
 * \author jan issac (jan.issac@gmail.com)
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/random/mersenne_twister.hpp>
#include <boost/shared_ptr.hpp>
#include <dbot/pose/pose_vector.h>
#include <dbrt/part_mesh_model.h>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <list>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <vector>

class KinematicsFromURDF
{
public:
    KinematicsFromURDF(const std::string& robot_description,
                       const std::string& robot_description_package_path,
                       const std::string& rendering_root_left,
                       const std::string& rendering_root_right,
                       const std::string& camera_frame_id,
                       const bool& use_camera_offset = false);

    ~KinematicsFromURDF();

    /// mutators ***************************************************************
    void set_joint_angles(const Eigen::VectorXd& joint_state);

    /// accessors **************************************************************
    Eigen::VectorXd get_link_position(int index);
    Eigen::Quaternion<double> get_link_orientation(int index);
    dbot::PoseVector get_link_pose(int index);

    std::vector<int> get_joint_order(const sensor_msgs::JointState& state);
    void get_part_meshes(
        std::vector<boost::shared_ptr<PartMeshModel>>& part_meshes);
    KDL::Tree get_tree();

    int num_joints();
    int num_links();
    std::string get_link_name(int idx);
    const std::vector<std::string>& get_joint_map() const;
    std::string get_root_frame_id();

    /// convenience ************************************************************
    Eigen::VectorXd sensor_msg_to_eigen(const sensor_msgs::JointState& angles);
    void print_joints();
    void print_links();

    // get the joint index in state array
    int name_to_index(const std::string& name);

private:
    void rename_camera_frame(const std::string& camera_frame,
                             urdf::Model& urdf);
    void inject_offset_joints_and_links(const std::string& camera_frame,
                                        urdf::Model& urdf);

    void check_size(int size);

    void compute_transforms();

    // std::string tf_correction_root_;
    std::string description_path_;

    // model as constructed form the robot urdf description
    urdf::Model urdf_;
    // KDL kinematic tree
    KDL::Tree kin_tree_;

    // maps joint indices to joint names and joint limits
    std::vector<std::string> joint_map_;

    // maps mesh indices to link names
    std::vector<std::string> mesh_names_;
    // maps link names to KDL frames
    std::map<std::string, KDL::Frame> frame_map_;

    // KDL segment map connecting link segments to joints
    KDL::SegmentMap segment_map_;
    // Forward kinematics solver
    KDL::TreeFkSolverPos_recursive* tree_solver_;

    // KDL copy of the joint state
    KDL::JntArray jnt_array_;
    // Contains Camera pose relative to base
    KDL::Frame cam_frame_;
    std::string cam_frame_name_;

    // rendering roots for left and right arm to exclude occluding head meshes
    std::string rendering_root_left_, rendering_root_right_;

    bool use_camera_offset_;
    dbot::PoseVector camera_offset_;
};
