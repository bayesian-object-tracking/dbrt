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

#pragma once

#include <Eigen/Dense>
#include <vector>

#include <memory>
#include <mutex>

#include <osr/euler_vector.hpp>
#include <osr/rigid_bodies_state.hpp>

// TODO: THERE IS A PROBLEM HERE BECAUSE WE SHOULD NOT DEPEND ON THIS FILE,
// SINCE IT IS IN A PACKAGE WHICH IS BELOW THIS PACKAGE.
#include <dbrt/kinematics_from_urdf.h>

namespace dbrt
{

template <int JointCount = Eigen::Dynamic, int BodyCount = Eigen::Dynamic>
class RobotState : public osr::RigidBodiesState<JointCount>
{
public:
    typedef osr::RigidBodiesState<JointCount> Base;
    typedef typename Base::Vector Vector;
    typedef typename Base::PoseVelocityBlock PoseVelocityBlock;

public:
    RobotState() : Base() {}
    template <typename T>
    RobotState(const Eigen::MatrixBase<T>& state_vector)
        : Base(state_vector)
    {
    }

    virtual ~RobotState() noexcept {}
    using Base::operator=;

public:
    /// \todo this function should not be named count, this is confusing
    virtual int count() const
    {
        CheckKinematics();
        return kinematics_->num_links();
    }


    virtual int count_parts() const
    {
        CheckKinematics();
        return kinematics_->num_links();
    }

    /// todo: why does this return a pose velocity vector, but not
    /// set the velocity?
    virtual osr::PoseVelocityVector component(int index) const
    {
        std::lock_guard<std::mutex> lock(*kinematics_mutex_);
        osr::PoseVelocityVector vector;
        vector.position() = position(index);
        vector.orientation() = euler_vector(index);

        return vector;
    }

    // TODO: SHOULD THIS FUNCITON BE IN HERE?
    void GetJointState(std::map<std::string, double>& joint_positions) const
    {
        joint_positions.clear();
        CheckKinematics();
        std::vector<std::string> joint_map = kinematics_->get_joint_map();
        for (std::vector<std::string>::const_iterator it = joint_map.begin();
             it != joint_map.end();
             ++it)
        {
            joint_positions[*it] = (*this)(it - joint_map.begin(), 0);
        }
    }

//    void recount(int new_count)
//    {
//        return this->resize(new_count);
//    }

private:
    virtual Vector position(const size_t& object_index = 0) const
    {
        assert(this->size() > 0);
        CheckKinematics();
        kinematics_->set_joint_angles(*this);

        Vector v = kinematics_->get_link_position(object_index);
        return v;
    }

    virtual osr::EulerVector euler_vector(const size_t& object_index = 0) const
    {
        assert(this->size() > 0);
        CheckKinematics();
        kinematics_->set_joint_angles(*this);

        osr::EulerVector v;
        v.quaternion(kinematics_->get_link_orientation(object_index));
        return v;
    }

    void CheckKinematics() const
    {
        if (!kinematics_)
        {
            std::cout << "kinematics not set" << std::endl;
            exit(-1);
        }
    }

public:
    static std::shared_ptr<std::mutex> kinematics_mutex_;
    static std::shared_ptr<KinematicsFromURDF> kinematics_;
};

template <int JointCount, int BodyCount>
std::shared_ptr<KinematicsFromURDF>
    RobotState<JointCount, BodyCount>::kinematics_;

template <int JointCount, int BodyCount>
std::shared_ptr<std::mutex>
    RobotState<JointCount, BodyCount>::kinematics_mutex_;

}
