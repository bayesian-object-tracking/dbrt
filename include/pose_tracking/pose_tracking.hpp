/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Max-Planck-Institute for Intelligent Systems,
 *                     University of Southern California
 *    Jan Issac (jan.issac@gmail.com)
 *
 *
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 */

/**
 * @author Jan Issac (jan.issac@gmail.com)
 * Max-Planck-Institute for Intelligent Systems,
 *   University of Southern California
 */


#ifndef POSE_TRACKING_POSE_TRACKING_HPP
#define POSE_TRACKING_POSE_TRACKING_HPP

#include <vector>
#include <string>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <fast_filtering/distributions/gaussian.hpp>
#include <fast_filtering/distributions/sum_of_deltas.hpp>
#include <fast_filtering/distributions/standard_gaussian.hpp>

#include <fast_filtering/states/rigid_body_system.hpp>
#include <fast_filtering/states/floating_body_system.hpp>

#include <fast_filtering/models/processes/damped_wiener_process.hpp>
#include <fast_filtering/models/processes/integrated_damped_wiener_process.hpp>

#include <fast_filtering/filters/stochastic/rao_blackwell_coordinate_particle_filter.hpp>

#include <pose_tracking/states/robot_state.hpp>
#include <pose_tracking/models/observers/image_observer_cpu.hpp>
#ifdef BUILD_GPU
#include <pose_tracking/models/observers/image_observer_gpu/image_observer_gpu.hpp>
#endif

#include <pose_tracking/utils/pcl_interface.hpp>
#include <pose_tracking/utils/ros_interface.hpp>
#include <pose_tracking//models/processes/brownian_object_motion.hpp>

namespace ff { enum { X = Eigen::Dynamic }; }

extern template class std::vector<size_t>;
extern template class std::vector<int>;
extern template class std::vector<float>;
extern template class std::vector<double>;
extern template class std::vector<std::string>;

// look for where this is actually being instantiated
extern template class pcl::PointCloud<pcl::PointXYZ>;

//// look for where these are actually being instantiated
extern template class Eigen::Matrix<double, ff::X, 1>;
extern template class Eigen::Matrix<double, 1, 1>;
extern template class Eigen::Matrix<double, 3, 1>;
extern template class Eigen::Matrix<double, 6, 1>;
extern template class Eigen::Matrix<float, 3, 1>;
extern template class Eigen::Matrix<float, 4, 1>;
//extern template class Eigen::Matrix<double, sf::X, sf::X>; // why is the operator= not being instantiated?

extern template class std::vector<RobotState<ff::X, ff::X> >;
extern template class std::vector<ff::FloatingBodySystem<ff::X> >;

extern template class std::vector<Eigen::Matrix<double, ff::X, 1> >;
extern template class std::vector<Eigen::Vector3i>;
extern template class std::vector<Eigen::Vector3f>;
extern template class std::vector<Eigen::Vector3d>;
extern template class std::vector<Eigen::Matrix4f>;

extern template class std::vector<std::vector<Eigen::Vector3d> >;
extern template class std::vector<std::vector<std::vector<int> > >;
extern template class std::vector<std::vector<size_t> >;
extern template class std::vector<pcl::PointCloud<pcl::PointXYZ> >;

extern template class ff::Gaussian<Eigen::Matrix<double, ff::X, 1> >;
extern template class ff::Gaussian<Eigen::Matrix<double, 1, 1> >;
extern template class ff::Gaussian<Eigen::Matrix<double, 3, 1> >;

extern template class ff::SumOfDeltas<RobotState<ff::X, ff::X> >;
extern template class ff::SumOfDeltas<ff::FloatingBodySystem<ff::X> >;
extern template class ff::SumOfDeltas<Eigen::Matrix<double, ff::X, 1> >;


extern template class ff::StandardGaussian<Eigen::Matrix<double, ff::X, 1> >;
extern template class ff::StandardGaussian<Eigen::Matrix<double, 1, 1> >;
extern template class ff::StandardGaussian<Eigen::Matrix<double, 3, 1> >;

extern template class ff::RigidBodySystem<ff::X>;
extern template class ff::FloatingBodySystem<ff::X>;
extern template class RobotState<ff::X, ff::X>;

extern template class ff::DampedWienerProcess<RobotState<ff::X, ff::X> >;
extern template class ff::DampedWienerProcess<Eigen::Matrix<double, 3, 1> >;

extern template class ff::IntegratedDampedWienerProcess<Eigen::Matrix<double, 6, 1> >;

extern template class ff::BrownianObjectMotion<ff::FloatingBodySystem<ff::X>, ff::X>;

extern template class ff::ImageObserverGPU<ff::FloatingBodySystem<ff::X> >;

extern template class ff::ImageObserverCPU<double, ff::FloatingBodySystem<ff::X> >;
extern template class ff::ImageObserverCPU<double, RobotState<ff::X, ff::X> >;

extern template class ff::RaoBlackwellCoordinateParticleFilter<
        ff::BrownianObjectMotion<ff::FloatingBodySystem<ff::X>,ff::X>,
        ff::RaoBlackwellObserver<
            ff::FloatingBodySystem<ff::X>,
            ff::ImageObserverCPU<double, ff::FloatingBodySystem<ff::X>, ff::X>::Observation
            > >;

extern template class ff::RaoBlackwellCoordinateParticleFilter<
        ff::DampedWienerProcess<RobotState<ff::X, ff::X> >,
        ff::ImageObserverCPU<double, RobotState<ff::X, ff::X>, ff::X> >;


extern template std::vector<ff::FloatingBodySystem<-1>::State>
pi::SampleTableClusters(const Eigen::Matrix<Eigen::Matrix<double,3,1>, -1, -1>&,
                        const size_t&);

extern template std::vector<ff::FloatingBodySystem<-1>::State>
pi::SampleTableClusters(const Eigen::Matrix<Eigen::Matrix<float,3,1>, -1, -1>&,
                        const size_t&);

extern template void
pi::Cluster(const pcl::PointCloud<pcl::PointXYZ>&,
            std::vector<pcl::PointCloud<pcl::PointXYZ> >&,
            const float&,
            const size_t&,
            const float&);

extern template void pi::PointsOnPlane(
        const std::vector<Eigen::Matrix<double, 3, 1> >& input_points,
        const size_t &input_rows,
        const size_t &input_cols,
        std::vector<Eigen::Matrix<double, 3, 1> >& output_points,
        size_t& output_rows, size_t& output_cols,
        Eigen::Matrix<double, 4, 1>& table_plane,
        const bool& keep_organized,
        const float& z_min,
        const float& z_max,
        const float& y_min,
        const float& y_max,
        const float& x_min,
        const float& x_max,
        const float& min_table_dist,
        const float& max_table_dist,
        const float& grid_size);

extern template void pi::PointsOnPlane(
        const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
        pcl::PointCloud<pcl::PointXYZ>& output_cloud,
        Eigen::Matrix<double, 4, 1>& table_plane,
        const bool& keep_organized,
        const float& z_min,
        const float& z_max,
        const float& y_min,
        const float& y_max,
        const float& x_min,
        const float& x_max,
        const float& min_table_dist,
        const float& max_table_dist,
        const float& grid_size);

extern template void pi::Pcl2Eigen(
        const pcl::PointCloud<pcl::PointXYZ> &point_cloud,
        std::vector<Eigen::Matrix<double, 3, 1> >& vector,
        size_t &n_rows, size_t &n_cols);

extern template std::vector<Eigen::Matrix<double, 3, 1> >
        pi::Pcl2Eigen(const pcl::PointCloud<pcl::PointXYZ> &point_cloud);

extern template void pi::Eigen2Pcl(
        const Eigen::Matrix<Eigen::Matrix<double, 3, 1>, -1, -1>& eigen,
        pcl::PointCloud<pcl::PointXYZ>& pcl);

extern template void pi::Eigen2Pcl(
        const std::vector<Eigen::Matrix<double, 3, 1> >& vector,
        const size_t &n_rows, const size_t &n_cols,
        pcl::PointCloud<pcl::PointXYZ> &point_cloud);

extern template void pi::Eigen2Pcl(
        const Eigen::Matrix<Eigen::Matrix<double, 3, 1>, -1, -1>& eigen,
        pcl::PointCloud<pcl::PointXYZ>& pcl);


extern template void ri::ReadParameter(const std::string& path,
                                       std::vector<std::string>& parameter,
                                       ros::NodeHandle node_handle);

extern template void ri::ReadParameter(const std::string& path,
                                       std::vector<double>& parameter,
                                       ros::NodeHandle node_handle);

extern template void ri::ReadParameter(const std::string& path,
                                       std::vector<std::vector<size_t> >& parameter,
                                       ros::NodeHandle node_handle);

extern template void ri::ReadParameter(const std::string& path,
                                       double& parameter,
                                       ros::NodeHandle node_handle);

extern template void ri::ReadParameter(const std::string& path,
                                       int& parameter,
                                       ros::NodeHandle node_handle);

extern template void ri::ReadParameter(const std::string& path,
                                       std::string& parameter,
                                       ros::NodeHandle node_handle);



#endif