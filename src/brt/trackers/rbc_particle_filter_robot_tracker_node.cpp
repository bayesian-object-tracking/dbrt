/*
 * This is part of the Bayesian Robot Tracking
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
 * \file rbc_particle_filter_robot_tracker_node.hpp
 * \date December 2015
 * \author Jan Issac (jan.issac@gmail.com)
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#include <memory>

#include <cv.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>

#include <dbot_ros/utils/ros_interface.hpp>
#include <dbot_ros/utils/ros_camera_data_provider.hpp>

#include <brt/states/robot_state.hpp>
#include <brt/utils/image_visualizer.hpp>
#include <brt/trackers/robot_tracker.hpp>
#include <dbot/util/rigid_body_renderer.hpp>
#include <brt/trackers/rbc_particle_filter_robot_tracker.hpp>
#include <dbot/tracker/builder/rb_observation_model_cpu_builder.hpp>
#include <brt/trackers/builder/rbc_particle_filter_robot_tracker_builder.hpp>

typedef brt::RbcParticleFilterRobotTracker Tracker;

class RobotTrackerNode
{
public:
    typedef Tracker::State State;
    typedef Tracker::Obsrv Obsrv;

public:
    /**
     * \brief Creates a TrackerNode
     */
    RobotTrackerNode()
        : priv_nh_("~"),
          first_time_(true),
          has_image_(false),
          has_joints_(false),
          tf_prefix_("MEAN")
    {
        ros::NodeHandle nh("~");
        pub_point_cloud_ =
            std::shared_ptr<ros::Publisher>(new ros::Publisher());

        *pub_point_cloud_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
            "/XTION/depth/points", 5);

        boost::shared_ptr<image_transport::ImageTransport> it(
            new image_transport::ImageTransport(node_handle_));
        pub_rgb_image_ = it->advertise("/XTION/depth/image_color", 5);

        // initialize the kinematics
        std::shared_ptr<KinematicsFromURDF> urdf_kinematics(
            new KinematicsFromURDF());

        /* ------------------------------ */
        /* - Parameters                 - */
        /* ------------------------------ */
        // tracker's main parameter container
        brt::RbcParticleFilterRobotTrackerBuilder::Parameters params;

        // camera data
        dbot::CameraData::Resolution resolution;
        std::string camera_info_topic;
        std::string depth_image_topic;
        int downsampling_factor;

        // parameter shorthand prefix
        std::string pre = "particle_filter/";

        // get filter parameters
        nh.getParam(pre + "use_gpu", params.use_gpu);

        nh.getParam(pre + "cpu/evaluation_count", params.cpu.evaluation_count);
        nh.getParam(pre + "cpu/update_rate", params.cpu.update_rate);
        nh.getParam(pre + "cpu/max_kl_divergence",
                    params.cpu.max_kl_divergence);

        nh.getParam(pre + "gpu/evaluation_count", params.gpu.evaluation_count);
        nh.getParam(pre + "gpu/update_rate", params.gpu.update_rate);
        nh.getParam(pre + "gpu/max_kl_divergence",
                    params.gpu.max_kl_divergence);

        nh.getParam(pre + "gpu/use_custom_shaders",
                    params.observation.use_custom_shaders);
        nh.getParam(pre + "gpu/vertex_shader_file",
                    params.observation.vertex_shader_file);
        nh.getParam(pre + "gpu/fragment_shader_file",
                    params.observation.fragment_shader_file);
        nh.getParam(pre + "gpu/geometry_shader_file",
                    params.observation.geometry_shader_file);

        params.tracker = params.use_gpu ? params.gpu : params.cpu;

        params.observation.sample_count = params.tracker.evaluation_count;

        nh.getParam(pre + "observation/occlusion/p_occluded_visible",
                    params.observation.occlusion.p_occluded_visible);
        nh.getParam(pre + "observation/occlusion/p_occluded_occluded",
                    params.observation.occlusion.p_occluded_occluded);
        nh.getParam(pre + "observation/occlusion/initial_occlusion_prob",
                    params.observation.occlusion.initial_occlusion_prob);

        nh.getParam(pre + "observation/kinect/tail_weight",
                    params.observation.kinect.tail_weight);
        nh.getParam(pre + "observation/kinect/model_sigma",
                    params.observation.kinect.model_sigma);
        nh.getParam(pre + "observation/kinect/sigma_factor",
                    params.observation.kinect.sigma_factor);
        params.observation.delta_time = 1. / 30.;

        // linear state transition parameters
        nh.getParam(pre + "joint_transition/joint_sigma",
                    params.state_transition.joint_sigma);
        nh.getParam(pre + "joint_transition/velocity_factor",
                    params.state_transition.velocity_factor);
        params.state_transition.joint_count = urdf_kinematics->num_joints();

        // camera parameters
        nh.getParam("camera_info_topic", camera_info_topic);
        nh.getParam("depth_image_topic", depth_image_topic);
        nh.getParam("downsampling_factor", downsampling_factor);
        nh.getParam("resolution/width", resolution.width);
        nh.getParam("resolution/height", resolution.height);

        /* ------------------------------ */
        /* - Setup camera data          - */
        /* ------------------------------ */
        // setup camera data
        auto camera_data_provider = std::shared_ptr<dbot::CameraDataProvider>(
            new dbot::RosCameraDataProvider(nh,
                                            camera_info_topic,
                                            depth_image_topic,
                                            resolution,
                                            downsampling_factor,
                                            2.0));
        dbot::CameraData camera_data(camera_data_provider);

        ros::Subscriber joint_states_sub =
            nh_.subscribe<sensor_msgs::JointState>(
                "/joint_states",
                1,
                &RobotTrackerNode::jointStateCallback,
                this);

        ros::Subscriber depth_image_sub = nh_.subscribe<sensor_msgs::Image>(
            depth_image_topic, 1, &RobotTrackerNode::depthImageCallback, this);

        //        //        Eigen::Matrix3d camera_matrix =
        //        Eigen::Matrix3d::Zero();
        //        //        // get the camera parameters
        //        //        while (camera_matrix.sum() == 0.0)
        //        //            camera_matrix =
        //        // ri::GetCameraMatrix<double>(camera_info_topic_, nh_,
        //        //                2.0);

        //        // initialize observation model
        //        //
        //        =================================================================================================
        //        // Read the URDF for the specific robot and get part meshes
        //        //        std::vector<boost::shared_ptr<PartMeshModel>>
        //        part_meshes_;
        //        //        urdf_kinematics->GetPartMeshes(part_meshes_);
        //        //        ROS_INFO("Number of part meshes %d",
        //        //        (int)part_meshes_.size());
        //        ROS_INFO("Number of links %d", urdf_kinematics->num_links());
        //        ROS_INFO("Number of joints %d",
        //        urdf_kinematics->num_joints());
        //        std::vector<std::string> joints =
        //        urdf_kinematics->GetJointMap();
        //        //    dbot::hf::PrintVector(joints);

        // get the name of the root frame
        root_ = urdf_kinematics->GetRootFrameID();

        // initialize the robot state publisher
        robot_state_publisher_ =
            std::make_shared<robot_state_pub::RobotStatePublisher>(
                urdf_kinematics->GetTree());

        State::kinematics_ = urdf_kinematics;

        while (!(has_joints_ & has_image_))
        {
            ROS_INFO("Waiting for joint angles and depth images: %d %d",
                     has_joints_,
                     has_image_);
            ros::spinOnce();
            usleep(10000);
        }

        std::vector<Eigen::VectorXd> initial_states_vectors =
            urdf_kinematics->GetInitialJoints(joint_state_copy_);
        std::vector<State> initial_states;
        for (auto state : initial_states_vectors)
        {
            initial_states.push_back(state);
        }

        /* ------------------------------ */
        /* - Create the tracker         - */
        /* ------------------------------ */
        auto tracker_builder = brt::RbcParticleFilterRobotTrackerBuilder(
            params, camera_data, urdf_kinematics);

        robot_renderer_ =
            dbot::RbObservationModelCpuBuilder<State>(
                params.observation, tracker_builder.create_object_model(), camera_data)
                .create_renderer();

        tracker_ = tracker_builder.build();
        tracker_->initialize(initial_states, urdf_kinematics);

        /* ------------------------------ */
        /* - Create and run tracker     - */
        /* - node                       - */
        /* ------------------------------ */
        //        RobotTrackerNode tracker_node(tracker, params.ori);
        ros::Subscriber subscriber = nh.subscribe(
            depth_image_topic, 1, &RobotTrackerNode::tracking_callback, this);

        ros::spin();
    }

    void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        ros_image_ = *msg;
        if (!has_image_) has_image_ = true;
        {
            // get the latest corresponding joint angles
            boost::mutex::scoped_lock lock(joint_state_mutex_);
            if (!first_time_)
            {
                joint_state_copy_ = joint_state_;
                has_joints_ = true;
            }
        }
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        boost::mutex::scoped_lock lock(joint_state_mutex_);
        joint_state_ = *msg;

        if (first_time_) first_time_ = false;
    }

    /**
     * \brief Tracking callback function which is invoked whenever a new image
     *        is available
     */
    void tracking_callback(const sensor_msgs::Image& ros_image)
    {
        auto image = ri::Ros2EigenVector<typename fl::Real>(
            ros_image, tracker_->camera_data().downsampling_factor());

        auto mean_ = tracker_->track(image);

        // DEBUG to see depth images
        std::vector<Eigen::Matrix3d> rotations(mean_.count());
        std::vector<Eigen::Vector3d> translations(mean_.count());
        for (size_t i = 0; i < rotations.size(); i++)
        {
            rotations[i] = mean_.component(i).orientation().rotation_matrix();
            translations[i] = mean_.component(i).position();
        }

        robot_renderer_->set_poses(rotations, translations);
        std::vector<int> indices;
        std::vector<float> depth;
        robot_renderer_->Render(camera_data().camera_matrix(),
                                image.rows(),
                                image.cols(),
                                indices,
                                depth);
        // image_viz_ = boost::shared_ptr<vis::ImageVisualizer>(new
        // vis::ImageVisualizer(image.rows(),image.cols()));
        vis::ImageVisualizer image_viz(image.rows(), image.cols());
                image_viz.set_image(image);
                image_viz.add_points(indices, depth);
                // image_viz.show_image("enchilada ", 500, 500, 1.0);

        std::map<std::string, double> joint_positions;
        mean_.GetJointState(joint_positions);

//        PVT(mean_);

        ros::Time t = ros::Time::now();
        // publish movable joints
        robot_state_publisher_->publishTransforms(
            joint_positions, t, tf_prefix_);
        // make sure there is a identity transformation between base of real
        // robot and estimated robot
        publishTransform(t, root_, tf::resolve(tf_prefix_, root_));
        // publish fixed transforms
        robot_state_publisher_->publishFixedTransforms(tf_prefix_);
        // publish image
        sensor_msgs::Image overlay;
        image_viz.get_image(overlay);
        publishImage(t, overlay);

        // publish point cloud
        Eigen::MatrixXd full_image = ri::Ros2Eigen<fl::Real>(ros_image, 1);
        Eigen::MatrixXd temp_camera_matrix = camera_data().camera_matrix();
        temp_camera_matrix.topLeftCorner(2, 3) *=
            double(camera_data().downsampling_factor());
        publishPointCloud(full_image, t, temp_camera_matrix);
    }

private:
    void publishImage(const ros::Time& time, sensor_msgs::Image& image);

    void publishTransform(const ros::Time& time,
                          const std::string& from,
                          const std::string& to);

    void publishPointCloud(const Eigen::MatrixXd &image,
                           const ros::Time& stamp,
                           const Eigen::MatrixXd& camera_matrix);

    const dbot::CameraData& camera_data() const;

private:
    ros::NodeHandle node_handle_;
    std::shared_ptr<Tracker> tracker_;

private:
    std::shared_ptr<robot_state_pub::RobotStatePublisher>
        robot_state_publisher_;
    std::shared_ptr<ros::Publisher> pub_point_cloud_;
    image_transport::Publisher pub_rgb_image_;
    std::string tf_prefix_;
    std::string root_;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    sensor_msgs::Image ros_image_;

    sensor_msgs::JointState joint_state_;
    sensor_msgs::JointState joint_state_copy_;
    boost::mutex joint_state_mutex_;

    ros::Subscriber subscriber_;

    bool first_time_;
    bool has_image_;
    bool has_joints_;
    std::shared_ptr<dbot::RigidBodyRenderer> robot_renderer_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rbc_particle_filter_robot_tracker");

    RobotTrackerNode rt;

    return 0;
}

/* ------------------------------ */
/* - Implementation             - */
/* ------------------------------ */
const dbot::CameraData& RobotTrackerNode::camera_data() const
{
    return tracker_->camera_data();
}

void RobotTrackerNode::publishImage(const ros::Time& time,
                                    sensor_msgs::Image& image)
{
    image.header.frame_id = tf::resolve(tf_prefix_, camera_data().frame_id());
    image.header.stamp = time;
    pub_rgb_image_.publish(image);
}

void RobotTrackerNode::publishTransform(const ros::Time& time,
                                        const std::string& from,
                                        const std::string& to)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setIdentity();
    br.sendTransform(tf::StampedTransform(transform, time, from, to));
}

void RobotTrackerNode::publishPointCloud(const Eigen::MatrixXd& image,
                                         const ros::Time& stamp,
                                         const Eigen::MatrixXd& camera_matrix)
{
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::PointCloud2Ptr points =
        boost::make_shared<sensor_msgs::PointCloud2>();
    points->header.frame_id = tf::resolve(tf_prefix_, camera_data().frame_id());
    points->header.stamp = stamp;
    points->width = image.cols();
    points->height = image.rows();
    points->is_dense = false;
    points->is_bigendian = false;
    points->fields.resize(3);
    points->fields[0].name = "x";
    points->fields[1].name = "y";
    points->fields[2].name = "z";
    int offset = 0;
    for (size_t d = 0; d < points->fields.size(); ++d, offset += sizeof(float))
    {
        points->fields[d].offset = offset;
        points->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
        points->fields[d].count = 1;
    }

    points->point_step = offset;
    points->row_step = points->point_step * points->width;

    points->data.resize(points->width * points->height * points->point_step);

    for (size_t u = 0, nRows = image.rows(), nCols = image.cols(); u < nCols;
         ++u)
        for (size_t v = 0; v < nRows; ++v)
        {
            float depth = image(v, u);
            if (depth != depth)  // || depth==0.0)
            {
                // depth is invalid
                memcpy(&points->data[v * points->row_step +
                                     u * points->point_step +
                                     points->fields[0].offset],
                       &bad_point,
                       sizeof(float));
                memcpy(&points->data[v * points->row_step +
                                     u * points->point_step +
                                     points->fields[1].offset],
                       &bad_point,
                       sizeof(float));
                memcpy(&points->data[v * points->row_step +
                                     u * points->point_step +
                                     points->fields[2].offset],
                       &bad_point,
                       sizeof(float));
            }
            else
            {
                // depth is valid
                float x = ((float)u - camera_matrix(0, 2)) * depth /
                          camera_matrix(0, 0);
                float y = ((float)v - camera_matrix(1, 2)) * depth /
                          camera_matrix(1, 1);
                memcpy(&points->data[v * points->row_step +
                                     u * points->point_step +
                                     points->fields[0].offset],
                       &x,
                       sizeof(float));
                memcpy(&points->data[v * points->row_step +
                                     u * points->point_step +
                                     points->fields[1].offset],
                       &y,
                       sizeof(float));
                memcpy(&points->data[v * points->row_step +
                                     u * points->point_step +
                                     points->fields[2].offset],
                       &depth,
                       sizeof(float));
            }
        }

    if (pub_point_cloud_->getNumSubscribers() > 0)
        pub_point_cloud_->publish(points);
}
