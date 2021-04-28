#include <IcpNode.h>


namespace icp_node {
    void IcpNode::callback(const PointCloud::ConstPtr &msg) {
        pcl::ScopeTime t1("callback");
        PointCloud::Ptr input_pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*msg.get(), *input_pt_cloud);
        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setMin(Eigen::Vector4f(-0.4, -0.4, -0.4, 1));
        box_filter.setMax(Eigen::Vector4f(0.4, 0.4, 0.4, 1));
        box_filter.setNegative(true);
        box_filter.setInputCloud(input_pt_cloud);
        box_filter.filter(*input_pt_cloud);

        box_filter.setMin(Eigen::Vector4f(-50, -50, -50, 1));
        box_filter.setMax(Eigen::Vector4f(50, 50, 50, 1));
        box_filter.setNegative(false);
        box_filter.setInputCloud(input_pt_cloud);
        box_filter.filter(*input_pt_cloud);

        processing(input_pt_cloud);
    }

    void IcpNode::processing(PointCloud::Ptr &msg_input_cloud) {
        pcl::transformPointCloud(*msg_input_cloud, *msg_input_cloud, global_transformation_m);

        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setLeafSize(0.5, 0.5, 0.5);
        voxel_filter.setInputCloud(msg_input_cloud);
        voxel_filter.filter(*msg_input_cloud);

        std::lock_guard<std::mutex> lock(processing_mutex);

        if (origin_pc == nullptr) {
            origin_pc = msg_input_cloud;
        } else {
//            PointCloud::Ptr result(new PointCloud);
//            Eigen::Matrix4f tmp_transformation = Eigen::Matrix4f::Identity();
//            pair_align(msg_input_cloud, origin_pc, result, tmp_transformation);
//            PointCloud::Ptr tmp_pc(new PointCloud);
//            pcl::transformPointCloud(*origin_pc, *tmp_pc, tmp_transformation.inverse());
//            tmp_pc->header.stamp = msg_input_cloud->header.stamp;
//            pub.publish(tmp_pc);
            // iterative approach
            pcl::transformPointCloud(*msg_input_cloud, *msg_input_cloud, global_transformation_m);
            Eigen::Matrix4f tmp_transformation = Eigen::Matrix4f::Identity();
            PointCloud::Ptr result(new PointCloud);
            PointCloud::Ptr tmp_pc(new PointCloud);
            pair_align(msg_input_cloud, origin_pc, result, tmp_transformation);
            global_transformation_m = tmp_transformation * global_transformation_m;
            pcl::transformPointCloud(*origin_pc, *tmp_pc, global_transformation_m.inverse());
            tmp_pc->header.stamp = msg_input_cloud->header.stamp;
            pub.publish(tmp_pc);
        }
    }

    void IcpNode::onInit() {
        pub = nh.advertise<PointCloud>("/uav1/points_icp/res", 1);
        sub = nh.subscribe("/uav1/os_cloud_nodelet/points", 1, &IcpNode::callback, this);
    }

    void IcpNode::pair_align(const PointCloud::Ptr &src,
                             const PointCloud::Ptr &tgt,
                             const PointCloud::Ptr &res,
                             Eigen::Matrix4f &final_transform) {
        pcl::ScopeTime t2("pair align");
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setLeafSize(0.5, 0.5, 0.5);

        voxel_filter.setInputCloud(tgt);
        voxel_filter.filter(*tgt);
        pcl::IterativeClosestPointNonLinear<PointT, PointT> icp;

        icp.setMaxCorrespondenceDistance(1.5);
        icp.setEuclideanFitnessEpsilon(0);
        icp.setTransformationEpsilon(0.00000001);
        icp.setMaximumIterations(15);

        // Align
        icp.setInputSource(src);
        icp.setInputTarget(tgt);

        icp.align(*res);

        final_transform = icp.getFinalTransformation();
    }

}

PLUGINLIB_EXPORT_CLASS(icp_node::IcpNode, nodelet::Nodelet);
