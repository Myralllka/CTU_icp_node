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

#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

namespace icp_node {

    using PointT = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointT>;
    using PointNormalT = pcl::PointNormal;
    using PointCloudWithNormals = pcl::PointCloud<PointNormalT>;

    struct PCD {
        PointCloud::Ptr cloud;
        std::string f_name;

        PCD() : cloud(new PointCloud) {};
    };

    struct PCDComparator {
        bool operator()(const PCD &p1, const PCD &p2) {
            return (p1.f_name < p2.f_name);
        }
    };

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
    };

    class MyPointRepresentation : public pcl::PointRepresentation<PointNormalT> {
        using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
    public:
        MyPointRepresentation() {
            // Define the number of dimensions
            nr_dimensions_ = 4;
        }

        // Override the copyToFloatArray method to define our feature vector
        void copyToFloatArray(const PointNormalT &p, float *out) const override {
            // < x, y, z, curvature >
            out[0] = p.x;
            out[1] = p.y;
            out[2] = p.z;
            out[3] = p.curvature;
        }
    };

    void pair_align(const PointCloud::Ptr &cloud_src,
                    const PointCloud::Ptr &cloud_tgt,
                    const PointCloud::Ptr &output,
                    Eigen::Matrix4f &final_transform,
                    bool down_sample);

}
