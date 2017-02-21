/*
 * This is part of the Bayesian Object Tracking (bot),
 * (https://github.com/bayesian-object-tracking)
 *
 * Copyright (c) 2016 Max Planck Society,
 * 				 Autonomous Motion Department,
 * 			     Institute for Intelligent Systems
 *
 * This Source Code Form is subject to the terms of the GNU General Public
 * License License (GNU GPL). A copy of the license can be found in the LICENSE
 * file distributed with this source code.
 */

/*
 * The RobotTransformsProvider originated from the RobotTransformsProvider at
 *   - git@github.com:ros/robot_state_publisher.git
 *   - from commit a06c4df
 * published under
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
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
 */

/**
 * \file robot_transforms_provider.cpp
 * \date Feb 2017
 * \author Jan Issac (jan.issac@gmail.com)
 * \author C. Garcia Cifuentes (c.garciacifuentes@gmail.com)
 */

#include <dbrt/robot_transforms_provider.h>

#include <geometry_msgs/TransformStamped.h>
#include <kdl/frames_io.hpp>
#include <tf2_kdl/tf2_kdl.h>
#include <tf_conversions/tf_kdl.h>

using namespace std;
using namespace ros;

namespace dbrt
{
RobotTransformsProvider::RobotTransformsProvider(const KDL::Tree& tree,
                                                 const urdf::Model& model)
    : model_(model)
{
    // walk the tree and add segments to segments_
    add_children(tree.getRootSegment());
}

// add children to correct maps
void RobotTransformsProvider::add_children(
    const KDL::SegmentMap::const_iterator segment)
{
    const std::string& root = GetTreeElementSegment(segment->second).getName();

    const std::vector<KDL::SegmentMap::const_iterator>& children =
        GetTreeElementChildren(segment->second);
    for (unsigned int i = 0; i < children.size(); i++)
    {
        const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
        SegmentPair s(
            GetTreeElementSegment(children[i]->second), root, child.getName());
        if (child.getJoint().getType() == KDL::Joint::None)
        {
            if (model_.getJoint(child.getJoint().getName()) &&
                model_.getJoint(child.getJoint().getName())->type ==
                    urdf::Joint::FLOATING)
            {
                ROS_INFO(
                    "Floating joint. Not adding segment from %s to %s. This TF "
                    "can not be published based on joint_states info",
                    root.c_str(),
                    child.getName().c_str());
            }
            else
            {
                segments_fixed_.insert(
                    make_pair(child.getJoint().getName(), s));
                ROS_DEBUG("Adding fixed segment from %s to %s",
                          root.c_str(),
                          child.getName().c_str());
            }
        }
        else
        {
            segments_.insert(make_pair(child.getJoint().getName(), s));
            ROS_DEBUG("Adding moving segment from %s to %s",
                      root.c_str(),
                      child.getName().c_str());
        }
        add_children(children[i]);
    }
}

// get moving transforms
void RobotTransformsProvider::get_transforms(
    const map<string, double>& joint_positions,
    const Time& time,
    const std::string& tf_prefix,
    std::vector<tf::StampedTransform>& tf_transforms) const
{
    ROS_DEBUG("Getting transforms for moving joints");
    tf_transforms.clear();
    get_moving_transforms_impl(joint_positions, time, tf_prefix, tf_transforms);
}
void RobotTransformsProvider::get_fixed_transforms(
    const ros::Time& time,
    const std::string& tf_prefix,
    std::vector<tf::StampedTransform>& tf_transforms) const
{
    ROS_DEBUG("Getting transforms for fixed joints");
    tf_transforms.clear();
    get_fixed_transforms_impl(time, tf_prefix, tf_transforms);
}
void RobotTransformsProvider::get_all_transforms(
    const map<string, double>& joint_positions,
    const ros::Time& time,
    const std::string& tf_prefix,
    std::vector<tf::StampedTransform>& tf_transforms) const
{
    ROS_DEBUG("Getting all transforms");
    tf_transforms.clear();
    get_moving_transforms_impl(joint_positions, time, tf_prefix, tf_transforms);
    get_fixed_transforms_impl(time, tf_prefix, tf_transforms);
}
void RobotTransformsProvider::get_moving_transforms_impl(
    const map<string, double>& joint_positions,
    const Time& time,
    const std::string& tf_prefix,
    std::vector<tf::StampedTransform>& tf_transforms) const
{
    tf::StampedTransform tf_transform;
    tf_transform.stamp_ = time;
    // loop over all joints
    for (map<string, double>::const_iterator jnt = joint_positions.begin();
         jnt != joint_positions.end();
         jnt++)
    {
        // find corresponding moving segment by joint name
        std::map<std::string, SegmentPair>::const_iterator seg_it =
            segments_.find(jnt->first);
        // ROS_INFO("Joint %s: %f", jnt->first.c_str(), jnt->second);
        if (seg_it != segments_.end())
        {
            SegmentPair seg_pair = seg_it->second;
            // find pose from joint value
            tf::transformKDLToTF(seg_pair.segment.pose(jnt->second),
                                 tf_transform);
            tf_transform.frame_id_ = tf::resolve(tf_prefix, seg_pair.root);
            tf_transform.child_frame_id_ = tf::resolve(tf_prefix, seg_pair.tip);
            tf_transforms.push_back(tf_transform);
        }
    }
}
void RobotTransformsProvider::get_fixed_transforms_impl(
    const ros::Time& time,
    const std::string& tf_prefix,
    std::vector<tf::StampedTransform>& tf_transforms) const
{
    tf::StampedTransform tf_transform;
    tf_transform.stamp_ =
        time;  // + ros::Duration(0.5); // future publish by 0.5 seconds
    // loop over all fixed segments
    for (map<string, SegmentPair>::const_iterator seg = segments_fixed_.begin();
         seg != segments_fixed_.end();
         seg++)
    {
        tf::transformKDLToTF(seg->second.segment.pose(0), tf_transform);
        tf_transform.frame_id_ = tf::resolve(tf_prefix, seg->second.root);
        tf_transform.child_frame_id_ = tf::resolve(tf_prefix, seg->second.tip);
        tf_transforms.push_back(tf_transform);
    }
}
}
