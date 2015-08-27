/*************************************************************************
This software allows for filtering in high-dimensional observation and
state spaces, as described in

M. Wuthrich, P. Pastor, M. Kalakrishnan, J. Bohg, and S. Schaal.
Probabilistic Object Tracking using a Range Camera
IEEE/RSJ Intl Conf on Intelligent Robots and Systems, 2013

In a publication based on this software pleace cite the above reference.


Copyright (C) 2014  Manuel Wuthrich

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*************************************************************************/
#include <ros/package.h>

#include <dbot/utils/profiling.hpp>

#include <state_filtering/trackers/object_tracker.hpp>
#include <state_filtering/utils/ros_interface.hpp>
#include <state_filtering/utils/object_file_reader.hpp>


#include <state_filtering/utils/cloud_visualizer.hpp>

#include <fl/model/process/orientation_transition_function.hpp>


#include <boost/filesystem.hpp>


MultiObjectTracker::MultiObjectTracker():
        node_handle_("~"),
        last_measurement_time_(std::numeric_limits<Scalar>::quiet_NaN())
{
    ri::ReadParameter("object_names", object_names_, node_handle_);
    ri::ReadParameter("downsampling_factor",downsampling_factor_, node_handle_);
    object_publisher_ =
          node_handle_.advertise<visualization_msgs::Marker>("object_model", 0);
}

void MultiObjectTracker::Initialize(
        std::vector<Eigen::VectorXd> initial_states,
        const sensor_msgs::Image& ros_image,
        Eigen::Matrix3d camera_matrix)
{
    boost::mutex::scoped_lock lock(mutex_);

    std::cout << "received " << initial_states.size()
                             << " intial states " << std::endl;
    // convert camera matrix and image to desired format
    camera_matrix.topLeftCorner(2,3) /= double(downsampling_factor_);
    Observation image = ri::Ros2Eigen<Scalar>(ros_image, downsampling_factor_);



    /// read parameters ********************************************************
    bool use_gpu;
    ri::ReadParameter("use_gpu", use_gpu, node_handle_);
    int evaluation_count;
    ri::ReadParameter("evaluation_count", evaluation_count, node_handle_);
    std::vector<std::vector<size_t> > sampling_blocks;
    ri::ReadParameter("sampling_blocks", sampling_blocks, node_handle_);
    double max_kl_divergence;
    ri::ReadParameter("max_kl_divergence", max_kl_divergence, node_handle_);

    int max_sample_count;
    ri::ReadParameter("max_sample_count", max_sample_count, node_handle_);

    double initial_occlusion_prob;
    ri::ReadParameter("initial_occlusion_prob",
                      initial_occlusion_prob, node_handle_);
    double p_occluded_visible;
    ri::ReadParameter("p_occluded_visible", p_occluded_visible, node_handle_);
    double p_occluded_occluded;
    ri::ReadParameter("p_occluded_occluded", p_occluded_occluded, node_handle_);

    double linear_acceleration_sigma;
    ri::ReadParameter("linear_acceleration_sigma",
                      linear_acceleration_sigma, node_handle_);
    double angular_acceleration_sigma;
    ri::ReadParameter("angular_acceleration_sigma",
                      angular_acceleration_sigma, node_handle_);
    double damping;
    ri::ReadParameter("damping", damping, node_handle_);

    double tail_weight;
    ri::ReadParameter("tail_weight", tail_weight, node_handle_);
    double model_sigma;
    ri::ReadParameter("model_sigma", model_sigma, node_handle_);
    double sigma_factor;
    ri::ReadParameter("sigma_factor", sigma_factor, node_handle_);

    double delta_time = 0.033;

    std::cout << "sampling blocks: " << std::endl;
    ff::hf::PrintVector(sampling_blocks);



    /// load object mesh *******************************************************
    std::vector<std::vector<Eigen::Vector3d> >
            vertices(object_names_.size());
    std::vector<std::vector<std::vector<int> > >
            triangle_indices(object_names_.size());
    for(size_t i = 0; i < object_names_.size(); i++)
    {
        std::string object_model_path = ros::package::getPath("state_filtering")
                        + "/object_models/"  + object_names_[i] + ".obj";
        ObjectFileReader file_reader;
        file_reader.set_filename(object_model_path);
        file_reader.Read();

        vertices[i] = *file_reader.get_vertices();
        triangle_indices[i] = *file_reader.get_indices();
    }

    /// compute object centers *************************************************
    centers_.resize(vertices.size());
    for(size_t i = 0; i < vertices.size(); i++)
    {
        centers_[i] = Eigen::Vector3d::Zero();
        for(size_t j = 0; j < vertices[i].size(); j++)
        {
            centers_[i] += vertices[i][j];
        }
        centers_[i] /= double(vertices[i].size());
    }

    /// switch coordinate system ***********************************************
    for(size_t i = 0; i < vertices.size(); i++)
    {
        for(size_t j = 0; j < vertices[i].size(); j++)
        {
            vertices[i][j] -= centers_[i];
        }
    }
    for(size_t i = 0; i < initial_states.size(); i++)
    {
        State state = initial_states[i];

        for(size_t j = 0; j < state.count(); j++)
        {
            state.component(j).position() +=
               state.component(j).orientation().rotation_matrix() * centers_[j];
        }

        initial_states[i] = state;
    }

    /// initialize cpu observation model ***************************************
    boost::shared_ptr<ObservationModel> observation_model;
#ifndef BUILD_GPU
    use_gpu = false;
#endif

    if(!use_gpu)
    {
        // cpu obseration model
        boost::shared_ptr<ff::KinectPixelObservationModel>
            kinect_pixel_observation_model(new ff::KinectPixelObservationModel(
                                                    tail_weight,
                                                    model_sigma,
                                                    sigma_factor));

        boost::shared_ptr<ff::OcclusionProcessModel> occlusion_process(
                            new ff::OcclusionProcessModel(p_occluded_visible,
                                                          p_occluded_occluded));

        boost::shared_ptr<ff::RigidBodyRenderer>
                renderer(new ff::RigidBodyRenderer(vertices, triangle_indices));

        observation_model = boost::shared_ptr<ObservationModelCPUType>(
                    new ObservationModelCPUType(camera_matrix,
                                        image.rows(),
                                        image.cols(),
                                        initial_states.size(),
                                        renderer,
                                        kinect_pixel_observation_model,
                                        occlusion_process,
                                        initial_occlusion_prob,
                                        delta_time));
    }

    /// initialize gpu observation model ***************************************
    else
    {
#ifdef BUILD_GPU

        /// \todo this is suboptimal to hardcode the path here.
        std::string vertex_shader_path =
                ros::package::getPath("dbot")
                + "/src/dbot/models/observation_models/"
                + "kinect_image_observation_model_gpu/shaders/"
                + "VertexShader.vertexshader";

        std::string fragment_shader_path =
                ros::package::getPath("dbot")
                + "/src/dbot/models/observation_models/"
                + "kinect_image_observation_model_gpu/shaders/"
                + "FragmentShader.fragmentshader";

        if(!boost::filesystem::exists(vertex_shader_path))
        {
            std::cout << "vertex shader does not exist at: "
                 << vertex_shader_path << std::endl;
            exit(-1);
        }
        if(!boost::filesystem::exists(fragment_shader_path))
        {
            std::cout << "fragment_shader does not exist at: "
                 << fragment_shader_path << std::endl;
            exit(-1);
        }

        // gpu obseration model
        boost::shared_ptr<ObservationModelGPUType>
                gpu_observation_model(new ObservationModelGPUType(
                                                 camera_matrix,
                                                 image.rows(),
                                                 image.cols(),
                                                 max_sample_count,
                                                 vertices,
                                                 triangle_indices,
                                                 vertex_shader_path,
                                                 fragment_shader_path,
                                                 initial_occlusion_prob,
                                                 delta_time,
                                                 p_occluded_visible,
                                                 p_occluded_occluded,
                                                 tail_weight,
                                                 model_sigma,
                                                 sigma_factor));


        observation_model = gpu_observation_model;
#endif
    }
    std::cout << "initialized observation omodel " << std::endl;

    /// initialize process model ***********************************************
    Eigen::MatrixXd linear_acceleration_covariance =
                            Eigen::MatrixXd::Identity(3, 3)
                            * pow(double(linear_acceleration_sigma), 2);
    Eigen::MatrixXd angular_acceleration_covariance =
                                   Eigen::MatrixXd::Identity(3, 3)
                                   * pow(double(angular_acceleration_sigma), 2);

    boost::shared_ptr<ProcessModel>
            process(new ProcessModel(delta_time, object_names_.size()));

    std::cout << "setting center shizzles " << std::endl;
    for(size_t i = 0; i < object_names_.size(); i++)
    {
        process->Parameters(i, Eigen::Vector3d::Zero(),
                               damping,
                               linear_acceleration_covariance,
                               angular_acceleration_covariance);
    }

    std::cout << "initialized process model " << std::endl;

    /// initialize filter ******************************************************
    filter_ = boost::shared_ptr<FilterType>(new FilterType(process,
                                                           observation_model,
                                                           sampling_blocks,
                                                           max_kl_divergence));

    std::vector<State > multi_body_samples(initial_states.size());
    for(size_t i = 0; i < multi_body_samples.size(); i++)
        multi_body_samples[i] = initial_states[i];

    filter_->set_particles(multi_body_samples);
    filter_->filter(image, ProcessModel::Input::Zero(object_names_.size()*6));

    filter_->resample(evaluation_count/sampling_blocks.size());

    /// convert to a differential reperesentation ******************************
//    State mean = filter_->belief().mean();
//    filter_->observation_model()->default_poses().recount(mean.count());
//    for(size_t i = 0; i < mean.count(); i++)
//    {
//        auto pose = filter_->observation_model()->default_poses().component(i);
//        auto delta = mean.component(i);
////        pose.orientation() = delta.orientation() * pose.orientation();
//        pose.position() = delta.position() + pose.position();
//    }

//    for(size_t i_part = 0; i_part < filter_->belief().size(); i_part++)
//    {
//        State& state = filter_->belief().location(i_part);
//        for(size_t i_obj = 0; i_obj < mean.count(); i_obj++)
//        {
//            state.component(i_obj).position() = state.component(i_obj).position() -
//                                        mean.component(i_obj).position();
////            state.component(i_obj).orientation() =
////                    state.component(i_obj).orientation() *
////                    mean.component(i_obj).orientation().inverse();
//        }
//    }

}





Eigen::VectorXd MultiObjectTracker::Filter(const sensor_msgs::Image& ros_image)
{
    boost::mutex::scoped_lock lock(mutex_);

    if(std::isnan(last_measurement_time_))
        last_measurement_time_ = ros_image.header.stamp.toSec();
    Scalar delta_time = ros_image.header.stamp.toSec() - last_measurement_time_;
    last_measurement_time_ = ros_image.header.stamp.toSec();
    std::cout << "actual delta time " << delta_time << std::endl;
    // convert image
    Observation image = ri::Ros2Eigen<Scalar>(ros_image, downsampling_factor_);

    /// filter *****************************************************************
    INIT_PROFILING;
    filter_->filter(image, ProcessModel::Input::Zero(object_names_.size()*6));
    MEASURE("-----------------> total time for filtering");


//    /// convert to a differential reperesentation ******************************
//    State mean_delta = filter_->belief().mean();
//    filter_->observation_model()->default_poses().recount(mean_delta.count());
//    for(size_t i = 0; i < mean_delta.count(); i++)
//    {
//        auto pose = filter_->observation_model()->default_poses().component(i);
//        auto delta = mean_delta.component(i);
//        pose.orientation() = delta.orientation() * pose.orientation();
//        pose.position() = delta.position() + pose.position();
//    }

//    for(size_t i_part = 0; i_part < filter_->belief().size(); i_part++)
//    {
//        State& state = filter_->belief().location(i_part);
//        for(size_t i_obj = 0; i_obj < mean_delta.count(); i_obj++)
//        {
//            state.component(i_obj).position() -=
//                                        mean_delta.component(i_obj).position();
//            state.component(i_obj).orientation() -=
//                                      mean_delta.component(i_obj).orientation();
//        }
//    }


    /// visualize the mean state ***********************************************
    State mean = filter_->belief().mean();
//    for(size_t i = 0; i < mean.count(); i++)
//    {
//        auto pose_0 = filter_->observation_model()->default_poses().component(i);
//        auto state = mean.component(i);

//        state.position() = state.position() + pose_0.position();
//        state.orientation() = state.orientation() * pose_0.orientation();
//    }

    // switch coordinate system
    for(size_t j = 0; j < mean.count(); j++)
    {
        mean.component(j).position() -=
                mean.component(j).orientation().rotation_matrix() * centers_[j];
    }

    for(size_t i = 0; i < object_names_.size(); i++)
    {
        std::string object_model_path =
         "package://state_filtering/object_models/" + object_names_[i] + ".obj";

        ri::PublishMarker(mean.component(i).homogeneous().cast<float>(),
                          ros_image.header, object_model_path, object_publisher_,
                          i, 1, 0, 0);
    }
    return mean;
}


/// just testing **************************************************
//    typedef fl::OrientationStateTransitionFunction TF;
//    TF tf;
//    tf.noise_matrix(TF::NoiseMatrix::Identity()*0.01);
//    tf.dynamics_matrix(TF::DynamicsMatrix::Identity()*0.9);

//    static Eigen::Matrix<double, 6, 1> orientation =
//                                            Eigen::Matrix<double, 6, 1>::Zero();
//    fl::Gaussian<TF::Noise> gaussian; gaussian.set_standard();
//    orientation = tf.state(orientation, gaussian.sample(), TF::Input::Zero());

//    ff::FreeFloatingRigidBodiesState<1> state;
//    state.orientation() = orientation.topRows(3);
//    state.position() = Eigen::Vector3d(0,0,1);

//    std::string object_model_path = "package://arm_object_models/objects/" + object_names_[0] + "/" + object_names_[0] + ".obj";
//    ri::PublishMarker(state.homogeneous_matrix().cast<float>(),
//                      ros_image.header, object_model_path, object_publisher_,
//                      1, 0, 1, 0);




/// ***************************************************************




















