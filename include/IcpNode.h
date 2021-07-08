//
// Created by myralllka on 29/03/2021.
//
#pragma once

#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <nodelet/nodelet.h>
#include <thread>
#include <utility>

#include <sstream>
#include <algorithm>
#include <string>
#include <iterator>

#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <pcl/common/time.h>
#include "aliases.h"

namespace icp_node {

    class IcpNode : public nodelet::Nodelet {
    public:
        void onInit() override;

        void callback_cloud(const pc_XYZ_t::ConstPtr &msg);

        void callback_pose(const nav_msgs::Odometry::ConstPtr &msg);

        void process_cloud(const pc_XYZ_t::ConstPtr &msg_input_cloud);

        [[maybe_unused]] void straight_forward_approach(const pc_XYZ_t::ConstPtr &msg_input_cloud);

        [[maybe_unused]] void iterative_approach(const pc_XYZ_t::ConstPtr &msg_input_cloud);

        void initialize_static_map(const std::string &filename);

    private:
        // it's a good practice to mark member variables to easily distinguish them from local variables
        // and avoid accidentally modifying them
        /* ros::NodeHandle nh; */  // you don't need to keep the node handle after initialization

        ros::Subscriber m_sub_pc;
        ros::Subscriber m_sub_position;

        ros::Publisher m_pub_orig;
        ros::Publisher m_pub_last;
        ros::Publisher m_pub_static_pc;
        tf2_ros::TransformBroadcaster m_tf_broadcaster;

        tf2_ros::Buffer m_tf_buffer;
        tf2_ros::TransformListener m_listener{m_tf_buffer};

        Eigen::Affine3d m_latest_gt_pose;
        Eigen::Affine3d m_origin_gt_pose;

        // always document what exactly is locked by a mutex (for your own good :)
        std::mutex m_processing_mutex;
        pc_XYZ_t::ConstPtr m_origin_cloud;
        pc_XYZ_t::ConstPtr m_previous_cloud;
        pc_XYZ_t::Ptr m_static_cloud;
        Eigen::Affine3d m_global_transformation = Eigen::Affine3d::Identity();

        Eigen::Affine3d pair_align(const pc_XYZ_t::ConstPtr &src, const pc_XYZ_t::ConstPtr &tgt, pc_XYZ_t &res);

        // consider using non-member functions instead of static methods if applicable to avoid having to define the method signature it in the header
        // also, non-member functions are usable by other objects as well (which makes sense in this case)
        /* static std::pair<double, double> compare_two_positions(const geometry_msgs::TransformStamped &source, */
        /*                                                        const geometry_msgs::TransformStamped &target); */

        // | --------------- Parameters, loaded from ROS -------------- |
        std::string m_uav_name;

        float m_icp_max_corr_dist;
        float m_icp_fitness_eps;
        float m_icp_tf_eps;
        int m_icp_max_its;


        const float m_apriori_map_voxelgrid_leaf_size = 0.5;
        const float m_apriori_map_tf_x = -467.0;
//        const float m_apriori_map_tf_x = 0.0;
        const float m_apriori_map_tf_y = -102.0;
//        const float m_apriori_map_tf_y = 0.0;
        const float m_apriori_map_tf_z = 3.0;
        const float m_apriori_map_tf_yaw = 0.0;
        const float m_apriori_map_correction_x = -3.0;
//        const float m_apriori_map_correction_x = 0.0;
        const float m_apriori_map_correction_y = 62.0;
//        const float m_apriori_map_correction_y = 0.0;
        const float m_apriori_map_correction_z = 0.0;

        std::string m_map_frame_id;
        std::string m_algorithm_type;

        bool m_apriori_map_initialized;
    };

}  // namespace icp_node