//
// Created by myralllka on 29/03/2021.
//
#pragma once

namespace icp_node {

    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

    class IcpNode : public nodelet::Nodelet {
    public:
        virtual void onInit();
        void callback(const pcl::PCLPointCloud2ConstPtr &msg);

    private:
        ros::Subscriber sub;
        ros::NodeHandle nh;
        ros::Publisher pub;
    };

}
