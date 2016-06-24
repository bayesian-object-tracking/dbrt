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
 * \file robot_transformer.hpp
 * \date April 2016
 * \author Cristina Garcia Cifuentes (c.garciacifuentes@gmail.com)
 */



#include <dbrt/robot_transformer.h>


namespace dbrt
{

RobotTransformer::RobotTransformer(
    const std::shared_ptr<robot_state_pub::RobotStatePublisher>& state_publisher):
    transforms_provider_(state_publisher),
    tf_transformer_(false),
    is_empty_(true),
    fake_time_(ros::Time(0)),
    fake_prefix_("") {}


void RobotTransformer::set_joints(const std::map<std::string, double>& joints)
{
    // Store the joints values
    joints_ = joints;

    // Lazy setting of the transformer (only when actually queried)
    tf_transformer_.clear();
    tf_transforms_.clear();
    is_empty_ = true;
}


const std::vector<tf::StampedTransform>& RobotTransformer::get_transforms() const
{
    if (joints_.empty())
    {
        ROS_WARN("Trying to get transforms, but joint values have not been set");
        return tf_transforms_;
    }
    if (tf_transforms_.empty())
    {
        // Use method from robot_state_pub::RobotStatePublisher to get tfs
        transforms_provider_->getAllTransforms(joints_, fake_time_,
            fake_prefix_, tf_transforms_);
    }
    return tf_transforms_;
}


void RobotTransformer::set_transformer_() const
{
    if (!is_empty_)
    {
        return;
    }

    const std::vector<tf::StampedTransform>& tf_transforms = get_transforms();
    // Fill in our transformer with these transforms
    for (std::size_t i = 0; i < tf_transforms.size(); ++i)
    {
        bool ok = tf_transformer_.setTransform(tf_transforms[i], "robot_transformer");
        if (!ok)
        {
            ROS_DEBUG("Problem setting tf from %s to %s",
                tf_transforms[i].frame_id_.c_str(),
                tf_transforms[i].child_frame_id_.c_str());
        }
    }
    is_empty_ = false;
}


void RobotTransformer::lookup_transform(const std::string& from,
    const std::string& to, tf::StampedTransform& tf_transform) const
{
    set_transformer_();
    std::string pfrom = tf::resolve(fake_prefix_, from);
    std::string pto = tf::resolve(fake_prefix_, to);
    try
    {
        tf_transformer_.lookupTransform(tf::resolve(fake_prefix_, from),
            tf::resolve(fake_prefix_, to), fake_time_, tf_transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("WEIRD! This should not happen... %s", ex.what());
        ros::Duration(1.0).sleep();
    }

}

} // end of namespace
