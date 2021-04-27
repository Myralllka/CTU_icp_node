#include <IcpNode.h>


namespace icp_node {
//    void IcpNode::callback(const PointCloud::ConstPtr &msg) {
    void IcpNode::callback(const PointCloud::ConstPtr &msg) {
        PointCloud::Ptr input_pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*msg.get(), *input_pt_cloud);
        box_filter.setInputCloud(input_pt_cloud);
        box_filter.filter(*input_pt_cloud);
        processing(input_pt_cloud);
    }

    void IcpNode::pair_align(const PointCloud::Ptr &src,
                             const PointCloud::Ptr &tgt,
                             const PointCloud::Ptr &res,
                             Eigen::Matrix4f &final_transform) {
        pcl::ScopeTime t1("pair_align");

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

//        PointCloud::Ptr reg_result = boost::make_shared<PointCloud>();
        icp.align(*res);

        final_transform = icp.getFinalTransformation();
        Eigen::Affine3f map2newmap;
        map2newmap = final_transform;
        const float tf_dist = map2newmap.translation().norm();
        std::cout << "\t correction: " << tf_dist << "m translation\n";

    }

    void IcpNode::processing(PointCloud::Ptr &input_cloud) {
//        while (ros::ok()) {
//        std::lock_guard<std::mutex> lock(processing_mutex);
        if (source == nullptr) {
            source = input_cloud;
        } else {
            PointCloud::Ptr result(new PointCloud);
            Eigen::Matrix4f tmp_transform = Eigen::Matrix4f::Identity();
            pair_align(input_cloud, source, result, tmp_transform);
            PointCloud::Ptr tmp_pc(new PointCloud);
            pcl::transformPointCloud(*source, *tmp_pc, tmp_transform.inverse());
            GlobalTransform *= tmp_transform;
            std::cout << "Transformation mat:\n" << tmp_transform.inverse() << std::endl;
            pub_source_transformed.publish(tmp_pc);
            pub_result.publish(result);
        }
//        }
    }

    void IcpNode::onInit() {
        //
        box_filter.setMin(Eigen::Vector4f(-0.4, -0.4, -0.4, 1));
        box_filter.setMax(Eigen::Vector4f(0.4, 0.4, 0.4, 1));
        box_filter.setNegative(true);
        //
        voxel_filter.setLeafSize(0.25, 0.25, 0.25);
        //
        pub_result = nh.advertise<PointCloud>("/uav1/points_icp/res", 1);
//        pub_source_transformed = nh.advertise<PointCloud>("/uav1/points_icp/src", 1);
        sub = nh.subscribe("/uav1/os_cloud_nodelet/points", 1, &IcpNode::callback, this);
//        std::thread data_processing(&IcpNode::processing, this);
//        data_processing.detach();
    }

}

PLUGINLIB_EXPORT_CLASS(icp_node::IcpNode, nodelet::Nodelet);
