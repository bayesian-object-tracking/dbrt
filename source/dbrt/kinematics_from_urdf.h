/*********************************************************************
 *
 *  Copyright (c) 2014, Jeannette Bohg - MPI for Intelligent System
 *  (jbohg@tuebingen.mpg.de)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Jeannette Bohg nor the names of MPI
 *     may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#ifndef POSE_TRACKING_INTERFACE_UTILS_KINEMATICS_FROM_URDF_HPP
#define POSE_TRACKING_INTERFACE_UTILS_KINEMATICS_FROM_URDF_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/random/mersenne_twister.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <list>
#include <vector>
#include <list>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

// tools
#include <dbrt/part_mesh_model.hpp>

#include <osr/pose_vector.hpp>

class KinematicsFromURDF
{
public:

    KinematicsFromURDF(const std::string& robot_description,
            const std::string& robot_description_package_path,
            const std::string& rendering_root_left,
            const std::string& rendering_root_right,
            const std::string& camera_frame_id,
            const bool& use_camera_offset=false);

    ~KinematicsFromURDF();

    // outputs a list of mesh model objects
    void get_part_meshes(std::vector<boost::shared_ptr<PartMeshModel> > &part_meshes);

    // Initialises the KDL data and specifically the camera pose
    void set_joint_angles(const Eigen::VectorXd& joint_state);

    // Get the position of the robot link with index idx
    Eigen::VectorXd get_link_position( int idx);

    // Get the orientation of the robot link with index idx
    Eigen::Quaternion<double> get_link_orientation( int idx);

    osr::PoseVector get_link_pose(int index);

    /// \todo this function shoudl not be in this class
    /// or at least it should be renamed
    // Convert Joint message to Eigen vector
    Eigen::VectorXd sensor_msg_to_eigen(const sensor_msgs::JointState &angles);

    std::vector<int> GetJointOrder(const sensor_msgs::JointState& state);

    // return the KDL kinematic tree
    KDL::Tree GetTree();

    // Get the number of joints
    int num_joints();

    int num_links();



    std::string GetLinkName(int idx);

    std::vector<std::string> GetJointMap();

    std::string GetRootFrameID();


    void print_joints();
    void print_links();
    // compute the transformations for all the links in one go

private:
    void check_size(int size);

    void ComputeLinkTransforms();

    // compute the camera frame for the current joint angles
    void SetCameraTransform();
    
    // get the joint index in state array
    int GetJointIndex(const std::string &name);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    //std::string tf_correction_root_;
    std::string description_path_;

    // model as constructed form the robot urdf description
    urdf::Model urdf_;
    // KDL kinematic tree
    KDL::Tree kin_tree_;

    // maps joint indices to joint names and joint limits
    std::vector<std::string> joint_map_;
    std::vector<float> lower_limit_;
    std::vector<float> upper_limit_;

    // maps mesh indices to link names
    std::vector<std::string> mesh_names_;
    // maps link names to KDL frames
    std::map<std::string, KDL::Frame> frame_map_;

    // KDL segment map connecting link segments to joints
    KDL::SegmentMap segment_map_;
    // Forward kinematics solver
    KDL::TreeFkSolverPos_recursive *tree_solver_;

    // KDL copy of the joint state
    KDL::JntArray jnt_array_;
    // Contains Camera pose relative to base
    KDL::Frame    cam_frame_;
    std::string   cam_frame_name_;

    // random generator for joint angle sampling
    boost::mt19937 generator_;

    // rendering roots for left and right arm to exclude occluding head meshes
    std::string rendering_root_left_, rendering_root_right_;

    bool use_camera_offset_;
    osr::PoseVector camera_offset_;
};

#endif
