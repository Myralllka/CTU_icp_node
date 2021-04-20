#include <IcpNode.h>

namespace icp_node {
//    void IcpNode::callback(const PointCloud::ConstPtr &msg) {
    void IcpNode::callback(const PointCloud::ConstPtr &msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::copyPointCloud(*msg.get(), *pt_cloud);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*pt_cloud, *pt_cloud, indices);
        if (source == nullptr) {
            source = pt_cloud;
        } else {
            PointCloud::Ptr result(new PointCloud);
            pair_align(source, pt_cloud, GlobalTransform, true);
            std::cout << GlobalTransform << std::endl;
            source = pt_cloud;
            printf("pub\n");
        }
    }



    void
    pair_align(const PointCloud::Ptr &cloud_src, const PointCloud::Ptr &cloud_tgt, Eigen::Matrix4f &final_transform,
               bool down_sample) {
        // Downsample for consistency and speed
        // \note enable this for large datasets
        PointCloud::Ptr src(new PointCloud);
        PointCloud::Ptr tgt(new PointCloud);
        pcl::VoxelGrid<PointT> grid;
        if (down_sample) {
            grid.setLeafSize(0.1, 0.1, 0.1);
            grid.setInputCloud(cloud_src);
            grid.filter(*src);

            grid.setInputCloud(cloud_tgt);
            grid.filter(*tgt);
        } else {
            src = cloud_src;
            tgt = cloud_tgt;
        }

        // Compute surface normals and curvature
        PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
        PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

        pcl::NormalEstimation<PointT, PointNormalT> norm_est;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        norm_est.setSearchMethod(tree);
        norm_est.setKSearch(30);

        norm_est.setInputCloud(src);
        norm_est.compute(*points_with_normals_src);
        pcl::copyPointCloud(*src, *points_with_normals_src);

        norm_est.setInputCloud(tgt);
        norm_est.compute(*points_with_normals_tgt);
        pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

        // Instantiate our custom point representation (defined above) ...
        MyPointRepresentation point_representation;
        // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
        float alpha[4] = {1.0, 1.0, 1.0, 1.0};
        point_representation.setRescaleValues(alpha);

        // Align
        pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
        reg.setTransformationEpsilon(1e-6);
        // Set the maximum distance between two correspondences (src<->tgt) to 10cm
        // Note: adjust this based on the size of your datasets
        reg.setMaxCorrespondenceDistance(0.1);
        // Set the point representation
        reg.setPointRepresentation(pcl::make_shared<const MyPointRepresentation>(point_representation));

        reg.setInputSource(points_with_normals_src);
        reg.setInputTarget(points_with_normals_tgt);

        // Run the same optimization in a loop and visualize the results
        Eigen::Matrix4f targetToSource;
        const PointCloudWithNormals::Ptr& reg_result = points_with_normals_src;
        reg.setMaximumIterations(30);

        PCL_INFO ("Iterations");

        // Estimate
        reg.setInputSource(points_with_normals_src);
        reg.align(*reg_result);

        final_transform = reg.getFinalTransformation().inverse();
    }

    void IcpNode::onInit() {
        pub = nh.advertise<PointCloud>("/uav1/points_icp", 1);
        sub = nh.subscribe("/uav1/os_cloud_nodelet/points", 1, &IcpNode::callback, this);
    }
}

PLUGINLIB_EXPORT_CLASS(icp_node::IcpNode, nodelet::Nodelet);