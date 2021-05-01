//
// Created by myralllka on 29/03/2021.
//
#pragma once

#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <nodelet/nodelet.h>

#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <nav_msgs/Odometry.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <pcl/common/time.h>

#include <thread>

namespace icp_node {

    using PointT = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointT>;

    class IcpNode : public nodelet::Nodelet {
    public:
        void onInit() override;

        void callback(const PointCloud::ConstPtr &msg);

        void callback_position(const nav_msgs::Odometry::ConstPtr &msg);

        void processing(PointCloud::Ptr &msg_input_cloud);

    private:
        ros::NodeHandle nh;

        ros::Subscriber sub;
        ros::Subscriber sub_position;

        ros::Publisher pub;
        ros::Publisher pub_iterative;
        ros::Publisher pub_error;

        tf2::Transform origin_position;
        tf2::Transform current_position;

        PointCloud::Ptr origin_pc;
        Eigen::Matrix4f global_transformation_m = Eigen::Matrix4f::Identity();
        tf2_ros::TransformBroadcaster tf_broadcaster;
        std::mutex processing_mutex;

        static void pair_align(const PointCloud::Ptr &src, const PointCloud::Ptr &tgt, const PointCloud::Ptr &res,
                               Eigen::Matrix4f &final_transform);

    };
}
