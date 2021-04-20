#include <IcpNode.h>
#include <pcl/common/time.h>

namespace icp_node {
//    void IcpNode::callback(const PointCloud::ConstPtr &msg) {
    void IcpNode::callback(const PointCloud::ConstPtr &msg) {
//        std::cout << "callback" << std::endl;
        pcl::ScopeTime t11("callback");
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*msg.get(), *input_pt_cloud);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*input_pt_cloud, *input_pt_cloud, indices);

        if (source == nullptr) {
            source = input_pt_cloud;
        } else {
            pcl::ScopeTime t12("callback_else_branch");
            PointCloud::Ptr result(new PointCloud);
            pair_align(source, input_pt_cloud, GlobalTransform, true);
//            std::cout << GlobalTransform << std::endl;
            Eigen::Affine3d affine(GlobalTransform.cast<double>());
//            tf::Transform transform;
//            tf::transformEigenToTF(affine, transform);

//            PointCloud::Ptr tfd_cloud = boost::make_shared<PointCloud>();
            pcl::transformPointCloud(*source, *source, GlobalTransform);
//            tfd_cloud->header.frame_id = m_map_frame_id;
            pub.publish(source);
//            source = tfd_cloud;
//            std::cout << "exit from callback" << std::endl;
        }
    }

    void
    pair_align(const PointCloud::Ptr &cloud_src, const PointCloud::Ptr &cloud_tgt, Eigen::Matrix4f &final_transform,
               bool down_sample) {
        pcl::ScopeTime t1("pair_align");
        // Downsample for consistency and speed
        // \note enable this for large datasets
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
        pcl::IterativeClosestPointNonLinear<PointT , PointT> reg;
//        reg.setTransformationEpsilon(1e-6);
        // Set the maximum distance between two correspondences (src<->tgt) to 10cm
        // Note: adjust this based on the size of your datasets
        reg.setMaxCorrespondenceDistance(10);
        reg.setInputSource(src);
        reg.setInputTarget(tgt);
        reg.setMaximumIterations(30);
        // Estimate

        PointCloud::Ptr reg_result = boost::make_shared<PointCloud>();
        reg.align(*reg_result);

        final_transform = reg.getFinalTransformation();
        Eigen::Affine3f map2newmap;
        map2newmap = final_transform;
        const float tf_dist = map2newmap.translation().norm();
//        const float tf_angl = pcl::anax_t().fromRotationMatrix(map2newmap.rotation()).angle();
//        std::cout << "Transformation correction\n" << transformation << "\n";
        std::cout << "\t correction: " << tf_dist << "m translation\n";


    }

    void IcpNode::onInit() {
        pub = nh.advertise<PointCloud>("/uav1/points_icp", 1);
        sub = nh.subscribe("/uav1/os_cloud_nodelet/points", 1, &IcpNode::callback, this);
    }
}

PLUGINLIB_EXPORT_CLASS(icp_node::IcpNode, nodelet::Nodelet);