#include <IcpNode.h>


namespace icp_node {
//    void IcpNode::callback(const PointCloud::ConstPtr &msg) {
    void IcpNode::callback(const PointCloud::ConstPtr &msg) {
//        std::cout << "callback" << std::endl;
        pcl::ScopeTime t11("callback");

        pcl::PointCloud<pcl::PointXYZ>::Ptr input_pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*msg.get(), *input_pt_cloud);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*input_pt_cloud, *input_pt_cloud, indices);
        box_filter.setInputCloud(input_pt_cloud);
        box_filter.filter(*input_pt_cloud);

        if (source == nullptr) {
            source = input_pt_cloud;
        } else {
            pcl::ScopeTime t12("callback_else_branch");
            PointCloud::Ptr result(new PointCloud);
            pair_align(source, input_pt_cloud, result, GlobalTransform);
            Eigen::Affine3d affine(GlobalTransform.cast<double>());
            pcl::transformPointCloud(*source, *source, GlobalTransform);
            pub.publish(source);
//            std::cout << "exit from callback" << std::endl;
        }
    }

    void IcpNode::pair_align(const PointCloud::Ptr &src,
                             const PointCloud::Ptr &tgt,
                             const PointCloud::Ptr res,
                             Eigen::Matrix4f &final_transform) {
        pcl::ScopeTime t1("pair_align");

        voxel_filter.setInputCloud(src);
        voxel_filter.filter(*src);

        voxel_filter.setInputCloud(tgt);
        voxel_filter.filter(*tgt);

        // Align
        icp.setInputSource(src);
        icp.setInputTarget(tgt);

        PointCloud::Ptr reg_result = boost::make_shared<PointCloud>();
        icp.align(*reg_result);

        final_transform = icp.getFinalTransformation();
        Eigen::Affine3f map2newmap;
        map2newmap = final_transform;
        const float tf_dist = map2newmap.translation().norm();
        std::cout << "\t correction: " << tf_dist << "m translation\n";

    }

    void IcpNode::onInit() {
        //
        box_filter.setMin(Eigen::Vector4f(-0.4, -0.4, -0.4, 1));
        box_filter.setMax(Eigen::Vector4f(0.4, 0.4, 0.4, 1));
        box_filter.setNegative(true);
        //
        voxel_filter.setLeafSize(0.25, 0.25, 0.25);
        //
//        icp.setMaxCorrespondenceDistance(10);
        icp.setMaximumIterations(30);
        //
        pub = nh.advertise<PointCloud>("/uav1/points_icp", 1);
        sub = nh.subscribe("/uav1/os_cloud_nodelet/points", 1, &IcpNode::callback, this);
    }
}

PLUGINLIB_EXPORT_CLASS(icp_node::IcpNode, nodelet::Nodelet);