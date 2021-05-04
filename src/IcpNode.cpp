#include <IcpNode.h>


namespace icp_node {
    void IcpNode::callback(const PointCloud::ConstPtr &msg) {
//        pcl::ScopeTime t1("callback");
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

    // You can use Eigen to calculate the point to point distance: (pt1-lt2).norm()
    double compute_distance(const geometry_msgs::TransformStamped &a1, const geometry_msgs::TransformStamped &a2) {
        auto x1 = static_cast<double>(a1.transform.translation.x);
        auto x2 = static_cast<double>(a2.transform.translation.x);
        auto y1 = static_cast<double>(a1.transform.translation.y);
        auto y2 = static_cast<double>(a2.transform.translation.y);
        auto z1 = static_cast<double>(a1.transform.translation.z);
        auto z2 = static_cast<double>(a2.transform.translation.z);
        return sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1)));
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
            origin_position = current_position;
        } else {
//            PointCloud::Ptr result(new PointCloud);
//            Eigen::Matrix4f tmp_transformation = Eigen::Matrix4f::Identity();
//            pair_align(msg_input_cloud, origin_pc, result, tmp_transformation);
//            PointCloud::Ptr tmp_pc(new PointCloud);
//            pcl::transformPointCloud(*origin_pc, *tmp_pc, tmp_transformation.inverse());
//            tmp_pc->header.stamp = msg_input_cloud->header.stamp;
//            pub.publish(tmp_pc);
            // iterative approach
            Eigen::Affine3d global_transformation_affine_m;
            Eigen::Matrix4f tmp_transformation = Eigen::Matrix4f::Identity();
            PointCloud::Ptr result(new PointCloud);
            PointCloud::Ptr tmp_pc(new PointCloud);

            pcl::transformPointCloud(*msg_input_cloud, *msg_input_cloud, global_transformation_m);
            pair_align(msg_input_cloud, origin_pc, result, tmp_transformation);

            global_transformation_m = tmp_transformation * global_transformation_m;
            pcl::transformPointCloud(*origin_pc, *tmp_pc, global_transformation_m.inverse());
            tmp_pc->header.stamp = msg_input_cloud->header.stamp;
//            pub.publish(tmp_pc);
            // affine transformation
            global_transformation_affine_m = global_transformation_m.cast<double>();

            geometry_msgs::TransformStamped msg;
            ros::Time msg_stamp;

            pcl_conversions::fromPCL(msg_input_cloud->header.stamp, msg_stamp);
            msg.header.frame_id = "world";
            msg.child_frame_id = "uav1/uav";
            msg.header.stamp = msg_stamp;
            msg.transform = tf2::eigenToTransform(global_transformation_affine_m.inverse()).transform;
            tf_broadcaster.sendTransform(msg);
            std::cout << current_position.header.frame_id << std::endl;
            // TODO: compare the msg with origin
            auto dist = compute_distance(current_position, msg);
            std::cout << "error in transform = " << dist << "m\n";
        }
    }

    void IcpNode::onInit() {
        pub = nh.advertise<PointCloud>("/uav1/points_icp/res", 1);
        sub = nh.subscribe("/uav1/os_cloud_nodelet/points", 1, &IcpNode::callback, this);
        sub_position = nh.subscribe("/uav1/ground_truth", 1, &IcpNode::callback_position, this);
    }

    void IcpNode::pair_align(const PointCloud::Ptr &src,
                             const PointCloud::Ptr &tgt,
                             const PointCloud::Ptr &res,
                             Eigen::Matrix4f &final_transform) {
//        pcl::ScopeTime t2("pair align");
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setLeafSize(0.5, 0.5, 0.5);

        voxel_filter.setInputCloud(tgt);
        voxel_filter.filter(*tgt);
        pcl::IterativeClosestPointNonLinear<PointT, PointT> icp;

        icp.setMaxCorrespondenceDistance(1);
        icp.setEuclideanFitnessEpsilon(0);
        icp.setTransformationEpsilon(0.00000001);
        icp.setMaximumIterations(30);

        // Align
        icp.setInputSource(src);
        icp.setInputTarget(tgt);

        icp.align(*res);

        final_transform = icp.getFinalTransformation();
    }

    void IcpNode::callback_position(const nav_msgs::Odometry::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(processing_mutex);
        current_position.header = msg->header;
        current_position.child_frame_id = msg->child_frame_id;
        current_position.transform.translation.x = msg->pose.pose.position.x;
        current_position.transform.translation.y = msg->pose.pose.position.y;
        current_position.transform.translation.z = msg->pose.pose.position.z;
        current_position.transform.rotation = msg->pose.pose.orientation;
    }

    bool IcpNode::transform_to_world(const std::string &input_frame_id,
                                     ros::Time stamp,
                                     Eigen::Affine3d &tf_out_affine_transformation) {
        try {
            const ros::Duration timeout(1.0 / 100.0);
            // Obtain transform from sensor into world frame
            geometry_msgs::TransformStamped transform;
            transform = tfBuffer.lookupTransform("world", input_frame_id, stamp, timeout);
            tf_out_affine_transformation = tf2::transformToEigen(transform.transform);
        }
        catch (tf2::TransformException &ex) {
            NODELET_WARN_THROTTLE(1.0, "[%s]: Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s",
                                  "icp_node", input_frame_id.c_str(),
                                  "world", ex.what());
            return false;
        }
        return true;

    }
}

PLUGINLIB_EXPORT_CLASS(icp_node::IcpNode, nodelet::Nodelet);
