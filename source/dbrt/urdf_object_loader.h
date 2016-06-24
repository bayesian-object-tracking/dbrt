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
 * \file urdf_object_model_loader.hpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <memory>

#include <dbot/common/object_model_loader.hpp>
#include <dbrt/kinematics_from_urdf.h>

namespace dbrt
{
class UrdfObjectModelLoader : public dbot::ObjectModelLoader
{
public:
    /**
     * \brief Creates a UrdfObjectModelLoader
     */
    UrdfObjectModelLoader(
        const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics);

    /**
     * \brief Loads the mesh from urdf kinematics
     */
    void load(
        std::vector<std::vector<Eigen::Vector3d>>& vertices,
        std::vector<std::vector<std::vector<int>>>& triangle_indices) const;

private:
    std::shared_ptr<KinematicsFromURDF> urdf_kinematics_;
};
}
