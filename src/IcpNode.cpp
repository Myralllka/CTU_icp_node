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
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(-0.4, -0.4, -0.4, 1));
        boxFilter.setMax(Eigen::Vector4f(0.4, 0.4, 0.4, 1));
        boxFilter.setNegative(true);
        boxFilter.setInputCloud(input_pt_cloud);
        boxFilter.filter(*input_pt_cloud);

        if (source == nullptr) {
            source = input_pt_cloud;
        } else {
            pcl::ScopeTime t12("callback_else_branch");
            PointCloud::Ptr result(new PointCloud);
            pair_align(source, input_pt_cloud, GlobalTransform, true);
            Eigen::Affine3d affine(GlobalTransform.cast<double>());
            pcl::transformPointCloud(*source, *source, GlobalTransform);
            pub.publish(source);
//            std::cout << "exit from callback" << std::endl;
        }
    }

    void IcpNode::pair_align(const PointCloud::Ptr &cloud_src,
                             const PointCloud::Ptr &cloud_tgt,
                             Eigen::Matrix4f &final_transform,
                             bool down_sample) {
        pcl::ScopeTime t1("pair_align");
        PointCloud::Ptr src(new PointCloud);
        PointCloud::Ptr tgt(new PointCloud);
        pcl::VoxelGrid<PointT> grid;
        if (down_sample) {
            grid.setLeafSize(0.25, 0.25, 0.25);
            grid.setInputCloud(cloud_src);
            grid.filter(*src);

            grid.setInputCloud(cloud_tgt);
            grid.filter(*tgt);
        } else {
            src = cloud_src;
            tgt = cloud_tgt;
        }
//        std::cout << src->width << std::endl;
//        std::cout << src->height << std::endl;
//        std::cout << src->size() << std::endl;
        // Align
        pcl::IterativeClosestPointNonLinear<PointT, PointT> reg;

        reg.setMaxCorrespondenceDistance(10);
        reg.setInputSource(src);
        reg.setInputTarget(tgt);
        reg.setMaximumIterations(30);

        PointCloud::Ptr reg_result = boost::make_shared<PointCloud>();
        reg.align(*reg_result);

        final_transform = reg.getFinalTransformation();
        Eigen::Affine3f map2newmap;
        map2newmap = final_transform;
        const float tf_dist = map2newmap.translation().norm();
        std::cout << "\t correction: " << tf_dist << "m translation\n";

    }

    void IcpNode::onInit() {
        pub = nh.advertise<PointCloud>("/uav1/points_icp", 1);
        sub = nh.subscribe("/uav1/os_cloud_nodelet/points", 1, &IcpNode::callback, this);
    }
}

PLUGINLIB_EXPORT_CLASS(icp_node::IcpNode, nodelet::Nodelet);