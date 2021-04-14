#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <nodelet/nodelet.h>

#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <IcpNode.h>

namespace icp_node {
    void IcpNode::callback(const pcl::PCLPointCloud2ConstPtr &msg) {
        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(msg);
        sor.setLeafSize(1, 1, 1);
        sor.filter(*cloud_filtered);

        pub.publish(cloud_filtered);
//        printf("Cloud: width = %d, height = %d\n", msg.get()->width, msg.get()->height);
        printf("pub");
    }

    void IcpNode::onInit() {
        pub = nh.advertise<PointCloud>("/uav1/points_vexen", 1);
        sub = nh.subscribe("/uav1/os_cloud_nodelet/points", 1, &IcpNode::callback, this);
    }
}

PLUGINLIB_EXPORT_CLASS(icp_node::IcpNode, nodelet::Nodelet);