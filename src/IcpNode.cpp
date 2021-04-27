#include <IcpNode.h>


namespace icp_node {
    void IcpNode::callback(const PointCloud::ConstPtr &msg) {
        PointCloud::Ptr input_pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*msg.get(), *input_pt_cloud);
        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setMin(Eigen::Vector4f(-0.4, -0.4, -0.4, 1));
        box_filter.setMax(Eigen::Vector4f(0.4, 0.4, 0.4, 1));
        box_filter.setNegative(true);
        box_filter.setInputCloud(input_pt_cloud);
        box_filter.filter(*input_pt_cloud);
        processing(input_pt_cloud);
    }

    void IcpNode::pair_align(const PointCloud::Ptr &src,
                             const PointCloud::Ptr &tgt,
                             const PointCloud::Ptr &res,
                             Eigen::Matrix4f &final_transform) {
        pcl::ScopeTime t1("pair_align");
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setLeafSize(0.25, 0.25, 0.25);
        voxel_filter.setInputCloud(src);
        voxel_filter.filter(*src);

        voxel_filter.setInputCloud(tgt);
        voxel_filter.filter(*tgt);
        pcl::IterativeClosestPointNonLinear<PointT, PointT> icp;

        icp.setMaxCorrespondenceDistance(4);
        icp.setMaximumIterations(30);

        // Align
        icp.setInputSource(src);
        icp.setInputTarget(tgt);

        icp.align(*res);

        final_transform = icp.getFinalTransformation();
    }

    void IcpNode::processing(PointCloud::Ptr &input_cloud) {
        std::lock_guard<std::mutex> lock(processing_mutex);
        if (source == nullptr) {
            source = input_cloud;
        } else {
            PointCloud::Ptr result(new PointCloud);
            Eigen::Matrix4f tmp_transform = Eigen::Matrix4f::Identity();
            pair_align(input_cloud, source, result, tmp_transform);
            PointCloud::Ptr tmp_pc(new PointCloud);

            pcl::transformPointCloud(*source, *tmp_pc, tmp_transform.inverse());
            tmp_pc->header.stamp = input_cloud->header.stamp;

            pub_source_transformed.publish(tmp_pc);
            pub_result.publish(result);
        }
    }

    void IcpNode::onInit() {
        pub_result = nh.advertise<PointCloud>("/uav1/points_icp/res", 1);
        pub_source_transformed = nh.advertise<PointCloud>("/uav1/points_icp/src", 1);
        sub = nh.subscribe("/uav1/os_cloud_nodelet/points", 1, &IcpNode::callback, this);
    }

}

PLUGINLIB_EXPORT_CLASS(icp_node::IcpNode, nodelet::Nodelet);
