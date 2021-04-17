#include <IcpNode.h>

namespace icp_node {
//    void IcpNode::callback(const PointCloud::ConstPtr &msg) {
    void IcpNode::callback(const PointCloud::ConstPtr &msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::copyPointCloud(*msg.get(), *pt_cloud);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*pt_cloud, *pt_cloud, indices);
        Eigen::Matrix4f pair_transform;
        if (source == nullptr) {
            source = pt_cloud;
        } else {
            PointCloud::Ptr result(new PointCloud);
            pair_align(source, pt_cloud, result, pair_transform, true);

            //transform current pair into the global transform
            pcl::transformPointCloud(*result, *result, GlobalTransform);

            //update the global transform
            GlobalTransform *= pair_transform;

            source = pt_cloud;
            pub.publish(result);
//        printf("Cloud: width = %d, height = %d\n", msg.get()->width, msg.get()->height);
            printf("pub");
        }
    }

    void pair_align(const PointCloud::Ptr &cloud_src,
                    const PointCloud::Ptr &cloud_tgt,
                    const PointCloud::Ptr &output,
                    Eigen::Matrix4f &final_transform,
                    bool down_sample) {
        //
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

        //
        // Instantiate our custom point representation (defined above) ...
        MyPointRepresentation point_representation;
        // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
        float alpha[4] = {1.0, 1.0, 1.0, 1.0};
        point_representation.setRescaleValues(alpha);

        //
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



        //
        // Run the same optimization in a loop and visualize the results
        Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
        PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
        reg.setMaximumIterations(2);
        for (int i = 0; i < 30; ++i) {
            PCL_INFO ("Iteration Nr. %d.\n", i);

            // save cloud for visualization purpose
            points_with_normals_src = reg_result;

            // Estimate
            reg.setInputSource(points_with_normals_src);
            reg.align(*reg_result);

            //accumulate transformation between each Iteration
            Ti = reg.getFinalTransformation() * Ti;

            //if the difference between this transformation and the previous one
            //is smaller than the threshold, refine the process by reducing
            //the maximal correspondence distance
            if (std::abs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
                reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

            prev = reg.getLastIncrementalTransformation();
        }

        //
        // Get the transformation from target to source
        targetToSource = Ti.inverse();

        //
        // Transform target back in source frame
        pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

        //add the source to the transformed target
        *output += *cloud_src;

//        final_transform = targetToSource;
        final_transform = Ti.inverse();
    }

    void IcpNode::onInit() {
        pub = nh.advertise<PointCloud>("/uav1/points_icp", 1);
        sub = nh.subscribe("/uav1/os_cloud_nodelet/points", 1, &IcpNode::callback, this);
    }
}

PLUGINLIB_EXPORT_CLASS(icp_node::IcpNode, nodelet::Nodelet);