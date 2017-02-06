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

/**
 * \file robot_transformer.cpp
 * \date April 2016
 * \author C. Garcia Cifuentes (c.garciacifuentes@gmail.com)
 */



#include <dbrt/robot_transformer.h>


namespace dbrt
{

RobotTransformer::RobotTransformer(
    const std::shared_ptr<robot_state_pub::RobotStatePublisher>& state_publisher):
    transforms_provider_(state_publisher),
    fake_time_(ros::Time(0)),
    fake_prefix_(""),
    lookup_provider_(false),
    is_empty_(true) {}


void RobotTransformer::set_joints(const std::map<std::string, double>& joints)
{
    // Store the joint values
    joints_ = joints;

    // Lazy setting of the lookup provider (only when actually queried)
    lookup_provider_.clear();
    transforms_.clear();
    is_empty_ = true;
}


const std::vector<tf::StampedTransform>& RobotTransformer::get_transforms() const
{
    if (joints_.empty())
    {
        ROS_WARN("Trying to get transforms, but joint values have not been set");
        return transforms_;
    }
    if (transforms_.empty())
    {
        // Use method from robot_state_pub::RobotStatePublisher to get tfs
        transforms_provider_->getAllTransforms(joints_, fake_time_,
            fake_prefix_, transforms_);
    }
    return transforms_;
}


void RobotTransformer::set_lookup_provider_() const
{
    if (!is_empty_)
    {
        return;
    }

    const std::vector<tf::StampedTransform>& transforms = get_transforms();
    // Fill in our lookup provider with these transforms
    for (std::size_t i = 0; i < transforms.size(); ++i)
    {
        bool ok = lookup_provider_.setTransform(transforms[i], "robot_transformer");
        if (!ok)
        {
            ROS_DEBUG("Problem setting transform %s <- %s",
                transforms[i].frame_id_.c_str(),
                transforms[i].child_frame_id_.c_str());
        }
    }
    is_empty_ = false;
}


void RobotTransformer::lookup_transform(const std::string& from,
    const std::string& to, tf::StampedTransform& tf_transform) const
{
    set_lookup_provider_();
    try
    {
        lookup_provider_.lookupTransform(tf::resolve(fake_prefix_, from),
            tf::resolve(fake_prefix_, to), fake_time_, tf_transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("WEIRD! This should not happen... %s", ex.what());
        ros::Duration(1.0).sleep();
    }

}

} // end of namespace
