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

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/time.h>

namespace icp_node {

    using PointT = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointT>;

    class IcpNode : public nodelet::Nodelet {
    public:
        void onInit() override;
        void callback(const PointCloud::ConstPtr &msg);
//        void callback(const pcl::PCLPointCloud2ConstPtr &msg);

    private:
        ros::Subscriber sub;
        ros::NodeHandle nh;
        ros::Publisher pub;
        PointCloud::Ptr source, target;
        Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
        pcl::CropBox<pcl::PointXYZ> box_filter;
        pcl::VoxelGrid<PointT> voxel_filter;
        pcl::IterativeClosestPointNonLinear<PointT, PointT> icp;
        void pair_align(const PointCloud::Ptr &src, const PointCloud::Ptr &tgt, const PointCloud::Ptr res,
                        Eigen::Matrix4f &final_transform);

    };
}
