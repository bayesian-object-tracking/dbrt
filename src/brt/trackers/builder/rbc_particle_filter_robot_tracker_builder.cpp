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



#include <dbot/tracker/builder/rbc_particle_filter_tracker_builder.hpp>
#include <brt/trackers/builder/rbc_particle_filter_robot_tracker_builder.hpp>

namespace brt
{
//RbcParticleFilterRobotTrackerBuilder::RbcParticleFilterRobotTrackerBuilder(
//    const Parameters& param,
//    const dbot::CameraData& camera_data,
//    const std::shared_ptr<KinematicsFromURDF>& urdf_kinematics)
//    : param_(param),
//      camera_data_(camera_data),
//      urdf_kinematics_(urdf_kinematics)
//{
//}

//std::shared_ptr<RbcParticleFilterRobotTracker>
//RbcParticleFilterRobotTrackerBuilder::build()
//{
//    auto object_model = create_object_model();
//    auto filter = create_filter(object_model, param_.tracker.max_kl_divergence);

//    auto tracker = std::make_shared<RbcParticleFilterRobotTracker>(
//        filter,
//        object_model,
//        camera_data_,
//        param_.tracker.evaluation_count);

//    return tracker;
//}

//auto RbcParticleFilterRobotTrackerBuilder::create_filter(
//    const dbot::ObjectModel& object_model,
//    double max_kl_divergence) -> std::shared_ptr<Filter>
//{
//    auto state_transition_model =
//        create_state_transition_model(param_.state_transition);

//    auto obsrv_model = create_obsrv_model(
//        param_.use_gpu, object_model, camera_data_, param_.observation);

//    auto sampling_blocks = create_sampling_blocks(urdf_kinematics_->num_joints());

//    auto filter = std::shared_ptr<Filter>(new Filter(state_transition_model,
//                                                     obsrv_model,
//                                                     sampling_blocks,
//                                                     max_kl_divergence));

//    return filter;
//}

//auto RbcParticleFilterRobotTrackerBuilder::create_state_transition_model(
//    const brt::RobotJointTransitionModelBuilder<
//        RbcParticleFilterRobotTrackerBuilder::State,
//        RbcParticleFilterRobotTrackerBuilder::Input>::Parameters& param) const
//    -> std::shared_ptr<StateTransition>
//{
//    brt::RobotJointTransitionModelBuilder<State, Input> process_builder(param);
//    std::shared_ptr<StateTransition> process = process_builder.build();

//    return process;
//}

//auto RbcParticleFilterRobotTrackerBuilder::create_obsrv_model(
//    bool use_gpu,
//    const dbot::ObjectModel& object_model,
//    const dbot::CameraData& camera_data,
//    const dbot::RbObservationModelBuilder<State>::Parameters& param) const
//    -> std::shared_ptr<ObservationModel>
//{
//    std::shared_ptr<ObservationModel> obsrv_model;

//    if (!use_gpu)
//    {
//        obsrv_model = dbot::RbObservationModelCpuBuilder<State>(
//                          param, object_model, camera_data)
//                          .build();
//    }
//    else
//    {
//#ifdef DBRT_BUILD_GPU
//        obsrv_model = dbot::RbObservationModelGpuBuilder<State>(
//                          param, object_model, camera_data)
//                          .build();
//#else
//        throw dbot::NoGpuSupportException();
//#endif
//    }

//    return obsrv_model;
//}

//dbot::ObjectModel RbcParticleFilterRobotTrackerBuilder::create_object_model()
//    const
//{
//    dbot::ObjectModel object_model;

//    object_model.load_from(std::shared_ptr<dbot::ObjectModelLoader>(
//                               new UrdfObjectModelLoader(urdf_kinematics_)),
//                           false);

//    return object_model;
//}

//std::vector<std::vector<size_t>>
//RbcParticleFilterRobotTrackerBuilder::create_sampling_blocks(int blocks) const
//{
//    int block_size = 1;
//    std::vector<std::vector<size_t>> sampling_blocks(blocks);
//    for (int i = 0; i < blocks; ++i)
//    {
//        for (int k = 0; k < block_size; ++k)
//        {
//            sampling_blocks[i].push_back(i * block_size + k);
//        }
//    }

//    return sampling_blocks;
//}
}
