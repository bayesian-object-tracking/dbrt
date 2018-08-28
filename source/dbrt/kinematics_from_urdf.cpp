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

#include <boost/random/normal_distribution.hpp>
#include <dbrt/kinematics_from_urdf.h>
#include <fl/util/profiling.hpp>

KinematicsFromURDF::KinematicsFromURDF(
    const std::string& robot_description,
    const std::string& robot_description_package_path,
    const std::string& rendering_root_left,
    const std::string& rendering_root_right,
    const std::string& camera_frame_id,
    const bool& use_camera_offset)
    : description_path_(robot_description_package_path),
      rendering_root_left_(rendering_root_left),
      rendering_root_right_(rendering_root_right),
      cam_frame_name_(camera_frame_id),
      use_camera_offset_(use_camera_offset)
{
    camera_offset_.setZero();

    // Initialize URDF object from robot description
    if (!urdf_.initString(robot_description)) ROS_ERROR("Failed to parse urdf");

    if (use_camera_offset_)
    {
        inject_offset_joints_and_links(cam_frame_name_, urdf_);
    }

    // set up kinematic tree from URDF
    if (!kdl_parser::treeFromUrdfModel(urdf_, kin_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return;
    }

    // create segment map for correct ordering of joints
    segment_map_ = kin_tree_.getSegments();
    boost::shared_ptr<const urdf::Joint> joint;
    joint_map_.resize(kin_tree_.getNrOfJoints());
    for (KDL::SegmentMap::const_iterator seg_it = segment_map_.begin();
         seg_it != segment_map_.end();
         ++seg_it)
    {
        if (seg_it->second.segment.getJoint().getType() != KDL::Joint::None)
        {
            joint = urdf_.getJoint(
                seg_it->second.segment.getJoint().getName().c_str());
            // check, if joint can be found in the URDF model of the
            // object/robot
            if (!joint)
            {
                ROS_FATAL(
                    "Joint '%s' has not been found in the URDF robot model! "
                    "Aborting ...",
                    joint->name.c_str());
                return;
            }
            // extract joint information
            if (joint->type != urdf::Joint::UNKNOWN &&
                joint->type != urdf::Joint::FIXED)
            {
                joint_map_[seg_it->second.q_nr] = joint->name;
            }
        }
    }

    // initialise kinematic tree solver
    tree_solver_ = new KDL::TreeFkSolverPos_recursive(kin_tree_);
}

void KinematicsFromURDF::rename_camera_frame(const std::string& camera_frame,
                                             urdf::Model& urdf)
{
    // rename joint child link name of the camera
    for (auto& joint_entry : urdf.joints_)
    {
        auto& joint = joint_entry.second;
        if (joint->child_link_name == camera_frame)
        {
            joint->child_link_name += "_DEFAULT";
            break;
        }
    }

    // rename camera link name
    for (auto& link_entry : urdf.links_)
    {
        auto& link = link_entry.second;
        if (link->name == camera_frame)
        {
            // rename link name
            link->name += "_DEFAULT";

            // add new entry using the new name
            urdf.links_[link->name] = link;

            // remove old entry mapped by old name
            auto camera_link_it = urdf.links_.find(camera_frame);
            urdf.links_.erase(camera_link_it);

            return;
        }
    }

    ROS_ERROR_STREAM("Camera frame ID" << camera_frame
                                       << " does not exist in URDF.");
}

void KinematicsFromURDF::inject_offset_joints_and_links(
    const std::string& camera_frame,
    urdf::Model& urdf)
{
    rename_camera_frame(camera_frame, urdf);

//    std::string camera_frame_ = "camera";
    std::vector<std::string> dofs = {camera_frame + "_X",
                                     camera_frame + "_Y",
                                     camera_frame + "_Z",
                                     camera_frame + "_PITCH",
                                     camera_frame + "_YAW",
                                     camera_frame + "_ROLL"};

    std::string current_parent_link = camera_frame + "_DEFAULT";

    for (int i = 0; i < dofs.size(); ++i)
    {
        // construct joint
        auto joint = boost::make_shared<urdf::Joint>();
        joint->name = dofs[i] + "_JOINT";
        joint->type = i < 3 ? urdf::Joint::PRISMATIC : urdf::Joint::REVOLUTE;
        joint->axis.x = double(i == 0 || i == 3);
        joint->axis.y = double(i == 1 || i == 4);
        joint->axis.z = double(i == 2 || i == 5);
        joint->child_link_name = dofs[i] + "_LINK";
        joint->parent_link_name = current_parent_link;

        // add joint
        urdf.joints_[joint->name] = joint;

        // update parent link name for next joint
        current_parent_link = joint->child_link_name;

        // last dof has no link
        if (i > 4) continue;

        auto link = boost::make_shared<urdf::Link>();
        link->name = dofs[i] + "_LINK";
        link->parent_joint = joint;

        // add link
        urdf.links_[link->name] = link;
    }

    // update camera root link child joints and links
    auto camera_root_link = urdf.links_[camera_frame + "_DEFAULT"];
    camera_root_link->child_joints.push_back(urdf.joints_[dofs[0] + "_JOINT"]);
    camera_root_link->child_links.push_back(urdf.links_[dofs[0] + "_LINK"]);

    // add leaf original camera frame
    auto camera_leaf_link = boost::make_shared<urdf::Link>();
    camera_leaf_link->name = camera_frame;
    camera_leaf_link->parent_joint = urdf.joints_[dofs[5] + "_JOINT"];
    urdf.links_[camera_frame] = camera_leaf_link;

    for (int i = 0; i < dofs.size() - 1; ++i)
    {
        auto link = urdf.links_[dofs[i] + "_LINK"];
        link->child_joints.push_back(urdf.joints_[dofs[i + 1] + "_JOINT"]);

        // ROLL is the last joint and the next link is the camera frame.
        // This means there is no ROLL link and there fore we do the next step
        // until PITCH link and YAW obtains the camera frame as child link
        if (i < dofs.size() - 2)
        {
            link->child_links.push_back(urdf.links_[dofs[i + 1] + "_LINK"]);
        }
        else
        {
            // Second las link has the camera frame as child link
            link->child_links.push_back(urdf.links_[camera_frame]);
        }
    }
}

KinematicsFromURDF::~KinematicsFromURDF()
{
    delete tree_solver_;
}

void KinematicsFromURDF::get_part_meshes(
    std::vector<boost::shared_ptr<PartMeshModel>>& part_meshes)
{
    // Load robot mesh for each link
    std::vector<boost::shared_ptr<urdf::Link>> links;
    urdf_.getLinks(links);
    std::string global_root = urdf_.getRoot()->name;
    for (unsigned i = 0; i < links.size(); i++)
    {
        // keep only the links descending from our root
        boost::shared_ptr<urdf::Link> tmp_link = links[i];
        while (tmp_link->name.compare(rendering_root_left_) == 0 &&
               tmp_link->name.compare(rendering_root_right_) == 0 &&
               tmp_link->name.compare(global_root) == 0)
        {
            tmp_link = tmp_link->getParent();
        }

        if (tmp_link->name.compare(global_root) == 0) continue;

        boost::shared_ptr<PartMeshModel> part_ptr(
            new PartMeshModel(links[i], description_path_, i, false));

        if (part_ptr->proper_)  // if the link has an actual mesh file to read
        {
            part_meshes.push_back(part_ptr);
            mesh_names_.push_back(part_ptr->get_name());
        }
    }
}

void KinematicsFromURDF::check_size(int size)
{
    int expected_size = kin_tree_.getNrOfJoints();

    if (expected_size != size)
    {
        std::cout << "a state of size: " << size
                  << " was passed. expected size: " << expected_size
                  << std::endl;
        exit(-1);
    }
}

void KinematicsFromURDF::set_joint_angles(const Eigen::VectorXd& joint_state)
{
    check_size(joint_state.size());

    // Internally, KDL array use Eigen Vectors
    if (jnt_array_.data.size() == 0 || !jnt_array_.data.isApprox(joint_state))
    {
        jnt_array_.data = joint_state.topRows(kin_tree_.getNrOfJoints());
        // Given the new joint angles, compute all link transforms in one go
        compute_transforms();
    }
}

void KinematicsFromURDF::compute_transforms()
{
    // get the transform from base to camera
    if (tree_solver_->JntToCart(jnt_array_, cam_frame_, cam_frame_name_) < 0)
        ROS_ERROR("TreeSolver returned an error for link %s",
                  cam_frame_name_.c_str());
    cam_frame_ = cam_frame_.Inverse();

    // loop over all segments to compute the link transformation
    for (KDL::SegmentMap::const_iterator seg_it = segment_map_.begin();
         seg_it != segment_map_.end();
         ++seg_it)
    {
        if (std::find(mesh_names_.begin(),
                      mesh_names_.end(),
                      seg_it->second.segment.getName()) != mesh_names_.end())
        {
            KDL::Frame frame;
            if (tree_solver_->JntToCart(
                    jnt_array_, frame, seg_it->second.segment.getName()) < 0)
                ROS_ERROR("TreeSolver returned an error for link %s",
                          seg_it->second.segment.getName().c_str());
            frame_map_[seg_it->second.segment.getName()] = cam_frame_ * frame;
        }
    }
}

Eigen::VectorXd KinematicsFromURDF::get_link_position(int index)
{
    Eigen::VectorXd pos(3);

    KDL::Frame& frame = frame_map_[mesh_names_[index]];
    pos << frame.p.x(), frame.p.y(), frame.p.z();

    return pos;
}

void KinematicsFromURDF::print_joints()
{
    std::cout << "robot joints: " << std::endl;
    for (size_t i = 0; i < joint_map_.size(); i++)
    {
        std::cout << "(" << i << " : " << joint_map_[i] << ")  " << std::endl;
    }
    std::cout << std::endl;
}

void KinematicsFromURDF::print_links()
{
    std::vector<boost::shared_ptr<urdf::Link>> links;
    urdf_.getLinks(links);

    std::cout << "robot links: " << std::endl;
    for (size_t i = 0; i < links.size(); i++)
    {
        std::cout << "(" << i << " : " << links[i]->name << ")  " << std::endl;
    }
    std::cout << std::endl;
}

Eigen::Quaternion<double> KinematicsFromURDF::get_link_orientation(int index)
{
    Eigen::Quaternion<double> quat;
    frame_map_[mesh_names_[index]].M.GetQuaternion(
        quat.x(), quat.y(), quat.z(), quat.w());

    return quat;
}

dbot::PoseVector KinematicsFromURDF::get_link_pose(int index)
{
    dbot::PoseVector pose_vector;
    pose_vector.orientation().quaternion(get_link_orientation(index));
    pose_vector.position() = get_link_position(index);

    return pose_vector;
}

// convert all joint angles to eigen
Eigen::VectorXd KinematicsFromURDF::joint_sensor_msg_to_eigen(
        const sensor_msgs::JointState::ConstPtr sensor_msg)
{
  sensor_msgs::JointState joint_state = *sensor_msg;  
  check_size(joint_state.position.size());
  
  Eigen::VectorXd eigen(joint_state.position.size());
  
  for (size_t i = 0; i < joint_state.position.size(); i++)
    {
      int joint_index = name_to_index(joint_state.name[i]);
      
      if (joint_index >= 0)
        {
	  eigen(joint_index) = joint_state.position[i];
        }
      else
        {
	  ROS_ERROR("i: %d, No joint index for %s",
		    int(i),
		    joint_state.name[i].c_str());
        }
    }
  
  return eigen;
}


Eigen::VectorXd KinematicsFromURDF::sensor_msg_to_eigen(
    const sensor_msgs::JointState& sensor_msg)
{
    sensor_msgs::JointState joint_state = sensor_msg;

    if (use_camera_offset_)
    {
        joint_state.name.push_back(cam_frame_name_ + "_X_JOINT");
        joint_state.name.push_back(cam_frame_name_ + "_Y_JOINT");
        joint_state.name.push_back(cam_frame_name_ + "_Z_JOINT");
        joint_state.name.push_back(cam_frame_name_ + "_ROLL_JOINT");
        joint_state.name.push_back(cam_frame_name_ + "_PITCH_JOINT");
        joint_state.name.push_back(cam_frame_name_ + "_YAW_JOINT");

        joint_state.position.push_back(0);
        joint_state.position.push_back(0);
        joint_state.position.push_back(0);
        joint_state.position.push_back(0);
        joint_state.position.push_back(0);
        joint_state.position.push_back(0);
    }

    check_size(joint_state.position.size());

    Eigen::VectorXd eigen(joint_state.position.size());

    for (size_t i = 0; i < joint_state.position.size(); i++)
    {
        int joint_index = name_to_index(joint_state.name[i]);

        if (joint_index >= 0)
        {
            eigen(joint_index) = joint_state.position[i];
        }
        else
        {
            ROS_ERROR("i: %d, No joint index for %s",
                      int(i),
                      joint_state.name[i].c_str());
        }
    }

    return eigen;
}

std::vector<int> KinematicsFromURDF::get_joint_order(
    const sensor_msgs::JointState& state)
{
    std::vector<int> order(state.name.size());
    for (int i = 0; i < state.name.size(); ++i)
    {
        order[i] = name_to_index(state.name[i]);
    }

    return order;
}

KDL::Tree KinematicsFromURDF::get_tree()
{
    return kin_tree_;
}

int KinematicsFromURDF::name_to_index(const std::string& name)
{
    for (unsigned int i = 0; i < joint_map_.size(); ++i)
    {
        if (joint_map_[i] == name) return i;
    }

    std::cout << "could not find joint with name " << name << std::endl;
    exit(-1);
    return -1;
}

std::string KinematicsFromURDF::get_link_name(int idx)
{
    return mesh_names_[idx];
}

int KinematicsFromURDF::num_joints()
{
    int n_joints = kin_tree_.getNrOfJoints();

    return n_joints;
}

int KinematicsFromURDF::num_links()
{
    return mesh_names_.size();
}

const std::vector<std::string>& KinematicsFromURDF::get_joint_map() const
{
    return joint_map_;
}

std::string KinematicsFromURDF::get_root_frame_id()
{
    return kin_tree_.getRootSegment()->first;
}
