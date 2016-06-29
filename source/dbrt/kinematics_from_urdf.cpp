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

#include <dbrt/kinematics_from_urdf.h>
#include <fl/util/profiling.hpp>
#include <boost/random/normal_distribution.hpp>

KinematicsFromURDF::KinematicsFromURDF(const std::string& robot_description,
        const std::string& robot_description_package_path,
        const std::string& rendering_root_left,
        const std::string& rendering_root_right,
        const std::string& camera_frame_id,
        const bool& use_camera_offset):
    description_path_(robot_description_package_path),
    rendering_root_left_(rendering_root_left),
    rendering_root_right_(rendering_root_right),
    cam_frame_name_(camera_frame_id),
    use_camera_offset_(use_camera_offset)
{

    camera_offset_.setZero();
    // Initialize URDF object from robot description
    if (!urdf_.initString(robot_description)) ROS_ERROR("Failed to parse urdf");

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
    lower_limit_.resize(kin_tree_.getNrOfJoints());
    upper_limit_.resize(kin_tree_.getNrOfJoints());
    for (KDL::SegmentMap::const_iterator seg_it = segment_map_.begin();
         seg_it != segment_map_.end();
         ++seg_it)
    {
        if (seg_it->second.segment.getJoint().getType() != KDL::Joint::None)
        {
            joint = urdf_.getJoint(
                        seg_it->second.segment.getJoint().getName().c_str());
            // check, if joint can be found in the URDF model of the object/robot
            if (!joint)
            {
                ROS_FATAL("Joint '%s' has not been found in the URDF robot model! "
                            "Aborting ...", joint->name.c_str());
                return;
            }
            // extract joint information
            if (joint->type != urdf::Joint::UNKNOWN &&
                    joint->type != urdf::Joint::FIXED)
            {
                joint_map_[seg_it->second.q_nr] = joint->name;
                lower_limit_[seg_it->second.q_nr] = joint->limits->lower;
                upper_limit_[seg_it->second.q_nr] = joint->limits->upper;
            }
        }
    }

    if(use_camera_offset_)
    {
        joint_map_.push_back("OFFSET_X");
        joint_map_.push_back("OFFSET_Y");
        joint_map_.push_back("OFFSET_Z");
        joint_map_.push_back("OFFSET_A");
        joint_map_.push_back("OFFSET_B");
        joint_map_.push_back("OFFSET_C");
    }

    // initialise kinematic tree solver
    tree_solver_ = new KDL::TreeFkSolverPos_recursive(kin_tree_);
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
            //	  std::cout << "link " << links[i]->name << " is descendant of "
            //<< tmp_link->name << std::endl;
            part_meshes.push_back(part_ptr);
            // Produces an index map for the links
            mesh_names_.push_back(part_ptr->get_name());
        }
    }
}

void KinematicsFromURDF::check_size(int size)
{
    int expected_size = kin_tree_.getNrOfJoints();
    if(use_camera_offset_) { expected_size += camera_offset_.size(); }

    if(expected_size != size)
    {
        std::cout << "a state of size: " << size
          << " was passed. expected size: " << expected_size << std::endl;
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
        ComputeLinkTransforms();
    }

    if(use_camera_offset_)
    {
        camera_offset_ = joint_state.bottomRows(camera_offset_.size());
    }
}

void KinematicsFromURDF::ComputeLinkTransforms()
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

Eigen::VectorXd KinematicsFromURDF::get_link_position(int idx)
{
    Eigen::VectorXd pos(3);

    KDL::Frame& frame = frame_map_[mesh_names_[idx]];
    pos << frame.p.x(), frame.p.y(), frame.p.z();

    if(use_camera_offset_)
    {
        pos = pos + camera_offset_.position();
    }
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

Eigen::Quaternion<double> KinematicsFromURDF::get_link_orientation(int idx)
{
    Eigen::Quaternion<double> quat;
    frame_map_[mesh_names_[idx]].M.GetQuaternion(
                quat.x(), quat.y(), quat.z(), quat.w());

    if(use_camera_offset_)
    {
        quat = camera_offset_.orientation().quaternion() * quat;
    }
    return quat;
}


osr::PoseVector KinematicsFromURDF::get_link_pose(int index)
{
    osr::PoseVector pose_vector;
    pose_vector.orientation().quaternion(get_link_orientation(index));
    pose_vector.position() = get_link_position(index);

    return pose_vector;
}

std::vector<Eigen::VectorXd> KinematicsFromURDF::GetInitialJoints(
        const sensor_msgs::JointState& angles)
{
    check_size(angles.position.size());

    std::vector<Eigen::VectorXd> samples;
    Eigen::VectorXd sample(num_joints());
    // loop over all joint and fill in KDL array
    for (std::vector<double>::const_iterator jnt = angles.position.begin();
         jnt != angles.position.end(); ++jnt)
    {
        int tmp_index = GetJointIndex(angles.name[jnt - angles.position.begin()]);

        if (tmp_index >= 0)
            sample(tmp_index) = *jnt;
        else
            ROS_ERROR("i: %d, No joint index for %s",
                      (int)(jnt - angles.position.begin()),
                      angles.name[jnt - angles.position.begin()].c_str());
    }
    samples.push_back(sample);

    return samples;
}

std::vector<int> KinematicsFromURDF::GetJointOrder(
        const sensor_msgs::JointState& state)
{
    std::vector<int> order(state.name.size());
    for (int i = 0; i < state.name.size(); ++i)
    {
        order[i] = GetJointIndex(state.name[i]);
    }

    return order;
}

KDL::Tree KinematicsFromURDF::GetTree()
{
    return kin_tree_;
}

int KinematicsFromURDF::GetJointIndex(const std::string& name)
{
    for (unsigned int i = 0; i < joint_map_.size(); ++i)
    {
        if (joint_map_[i] == name) return i;
    }

    std::cout << "could not find joint with name " << name << std::endl;
    exit(-1);
    return -1;
}

std::string KinematicsFromURDF::GetLinkName(int idx)
{
    return mesh_names_[idx];
}

int KinematicsFromURDF::num_joints()
{
    int n_joints  = kin_tree_.getNrOfJoints();
    if(use_camera_offset_ == true)
    {
        n_joints += camera_offset_.size();
    }
    return n_joints;
}

int KinematicsFromURDF::num_links()
{
    return mesh_names_.size();
}

std::vector<std::string> KinematicsFromURDF::GetJointMap()
{
    return joint_map_;
}

std::string KinematicsFromURDF::GetRootFrameID()
{
    return kin_tree_.getRootSegment()->first;
}
