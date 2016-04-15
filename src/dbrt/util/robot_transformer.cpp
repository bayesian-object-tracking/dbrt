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



#include <dbrt/util/robot_transformer.hpp>


namespace dbrt
{

RobotTransformer::RobotTransformer(
    const std::shared_ptr<robot_state_pub::RobotStatePublisher>& state_publisher):
    transforms_provider_(state_publisher),
    tf_transformer_(false),
    is_empty_(true) {}


void RobotTransformer::set_joints(
    const std::map<std::string, double>& joints, const ros::Time& time,
    const std::string& tf_prefix)
{
    // Store the joints values
    joints_ = joints;

    // These guys will be needed later to get transforms and query our transformer
    tf_prefix_ = tf_prefix;
    time_ = time; //ros::Time.now();

    // Lazy init if the transformer (only when actually queried)
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
        transforms_provider_->getAllTransforms(joints_, time_, tf_prefix_, tf_transforms_);
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
            ROS_DEBUG("Problem setting tf from %s to %s", tf_transforms[i].frame_id_.c_str(),
                tf_transforms[i].child_frame_id_.c_str());
        }
    }
    is_empty_ = false;
}


void RobotTransformer::lookup_transform(const std::string& from,
    const std::string& to, tf::StampedTransform& tf_transform) const
{
    set_transformer_();
    tf_transformer_.lookupTransform(tf::resolve(tf_prefix_, from),
        tf::resolve(tf_prefix_, to), time_, tf_transform);
}

} // end of namespace
