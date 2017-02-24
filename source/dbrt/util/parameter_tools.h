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
 * \file parameter_tools.h
 * \date August 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <dbot_ros/util/ros_interface.h>
#include <dbrt/kinematics_from_urdf.h>
#include <map>
#include <string>
#include <vector>

namespace dbrt
{
typedef std::vector<std::map<std::string, std::vector<std::string>>>
    SamplingBlocksDefinition;

inline std::vector<std::vector<int>> definition_to_sampling_block(
    const SamplingBlocksDefinition& definition,
    const std::shared_ptr<KinematicsFromURDF>& kinematics)
{
    std::vector<std::vector<int>> sampling_blocks;
    for (auto block_definition : definition)
    {
        std::vector<int> block_vector;
        for (auto joint_name : block_definition.begin()->second)
        {
            block_vector.push_back(kinematics->name_to_index(joint_name));
        }
        sampling_blocks.push_back(block_vector);
    }

    return sampling_blocks;
}

inline SamplingBlocksDefinition merge_sampling_block_definitions(
    const SamplingBlocksDefinition& definition_A,
    const SamplingBlocksDefinition& definition_B,
    const std::string& prefixB)
{
    SamplingBlocksDefinition merged = definition_A;

    for (auto block_definition_B : definition_B)
    {
      std::map<std::string, std::vector<std::string>> prefixed_block_definition_B;
      for (auto block : block_definition_B)
      {
	for (auto value : block.second)
	{
	  prefixed_block_definition_B[block.first].push_back(prefixB + value);
	}
      }

      auto block_B = prefixed_block_definition_B.begin();
        bool added = false;
        for (auto& block_definition_A : merged)
        {
            auto block_A = block_definition_A.begin();
            if (block_A->first == block_B->first)
            {
	       block_A->second.insert(std::end(block_A->second),
                                      std::begin(block_B->second),
                                      std::end(block_B->second));
               added = true;
            }
        }
        if (!added)
        {
            merged.push_back(prefixed_block_definition_B);
        }
    }

    return merged;
}

inline std::map<std::string, double> read_maps_from_map_list(
    const std::string& parameter,
    ros::NodeHandle& nh)
{
    std::map<std::string, double> parameter_map;

    auto map_list =
        ri::read<std::vector<std::map<std::string, double>>>(parameter, nh);

    for (auto map : map_list)
    {
        parameter_map.insert(map.begin(), map.end());
    }

    return parameter_map;
}

inline std::vector<double> extract_ordered_values(
    const std::map<std::string, double>& parameter_map,
    const std::shared_ptr<KinematicsFromURDF>& kinematics)
{
    std::vector<double> ordered_values(parameter_map.size());
    for (auto entry : parameter_map)
    {
        std::string joint_name = entry.first;
        if (kinematics->name_to_index(joint_name) > ordered_values.size())
        {
            ROS_ERROR(
                "Joint index exceeds number of read joints. "
                "Did you forget to merge joint lists?");
            exit(-1);
        }
        ordered_values[kinematics->name_to_index(joint_name)] = entry.second;
    }

    return ordered_values;
}

inline void insert_map_with_prefixed_keys(
    const std::map<std::string, double>& new_values, 
    const std::string& prefix, 
    std::map<std::string, double>& destination){
  for (auto new_value: new_values){
    destination[prefix + new_value.first] = new_value.second;
  }
}
}
