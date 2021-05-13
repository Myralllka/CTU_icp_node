#include <IcpNode.h>
// Matous code https://mrs.felk.cvut.cz/gitlab/vrbamato/uav_detect/blob/ouster/src/pcl_selfloc_nodelet.cpp#L420
//

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

        box_filter.setMin(Eigen::Vector4f(-40, -40, -40, 1));
        box_filter.setMax(Eigen::Vector4f(40, 40, 40, 1));
        box_filter.setNegative(false);
        box_filter.setInputCloud(input_pt_cloud);
        box_filter.filter(*input_pt_cloud);

        processing(input_pt_cloud);
    }

    void IcpNode::processing(PointCloud::Ptr &msg_input_cloud) {
        ros::Time msg_stamp;
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setLeafSize(0.5, 0.5, 0.5);
        voxel_filter.setInputCloud(msg_input_cloud);
        voxel_filter.filter(*msg_input_cloud);

        std::lock_guard<std::mutex> lock(processing_mutex);
        if (origin_pc == nullptr) {
            origin_pc = msg_input_cloud;
            previous_pc = msg_input_cloud;
            origin_position = current_position;
            origin_position.child_frame_id = "world/local_origin";
        } else {
            Eigen::Matrix4f tmp_transformation = Eigen::Matrix4f::Identity();
            PointCloud::Ptr result(new PointCloud);
            PointCloud::Ptr tmp_pc(new PointCloud);
            // Find the `tmp_transformation` - transformation from the msg to the previous message pc
            pair_align(msg_input_cloud, previous_pc, result, tmp_transformation);
            // global - transformation from current position to the origin
            global_transformation_m = tmp_transformation * global_transformation_m;
            pcl::transformPointCloud(*origin_pc, *tmp_pc, global_transformation_m.inverse());
            tmp_pc->header.stamp = msg_input_cloud->header.stamp;
            pub.publish(tmp_pc);

            // TF Broadcaster
            geometry_msgs::TransformStamped msg;
            pcl_conversions::fromPCL(msg_input_cloud->header.stamp, msg_stamp);
            msg.header.frame_id = "uav1/fcu";
            msg.child_frame_id = "uav1/uav";
            msg.header.stamp = msg_stamp;
            Eigen::Affine3d global_transformation_affine;
            global_transformation_affine = global_transformation_m.cast<double>();
            msg.transform = tf2::eigenToTransform(global_transformation_affine.inverse()).transform;
            tf_broadcaster.sendTransform(msg);
            geometry_msgs::TransformStamped from_origin_to_current_gt_transformation;

            get_transformation_to_frame("world/local_origin",
                                        current_position.child_frame_id,
                                        current_position.header.stamp,
                                        from_origin_to_current_gt_transformation);

            auto epsilon = compare_two_positions(
                    tf2::eigenToTransform(global_transformation_affine.inverse()),
                    from_origin_to_current_gt_transformation);

            std::cout << "\ttransformation error is: " << epsilon.first << "m.\n"
                      << "\trotation error is: " << epsilon.second << "deg.\n";
            previous_pc = msg_input_cloud;
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

        icp.setMaxCorrespondenceDistance(0.001);
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

    //    Listener for transformations between frames
    //    http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29
    void
    IcpNode::get_transformation_to_frame(const std::string &input_frame_id,
                                         const std::string &destination_frame_id,
                                         ros::Time stamp,
                                         geometry_msgs::TransformStamped &tf_out_transformation) {
        try {
            const ros::Duration timeout(1.0 / 100.0);
            // Obtain transform from sensor into world frame
            geometry_msgs::TransformStamped transform;
            tf_out_transformation = tfBuffer.lookupTransform(input_frame_id, input_frame_id, stamp, timeout);
        }
        catch (tf2::TransformException &ex) {
            NODELET_WARN_THROTTLE(1.0, "[%s]: Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s",
                                  "icp_node", input_frame_id.c_str(),
                                  destination_frame_id.c_str(), ex.what());
        }
    }

    std::pair<double, double> IcpNode::compare_two_positions(const geometry_msgs::TransformStamped &source,
                                                             const geometry_msgs::TransformStamped &target) {
        // Quaternion rotation http://wiki.ros.org/tf2/Tutorials/Quaternions#Relative_rotations
        // http://docs.ros.org/en/jade/api/tf2/html/classtf2_1_1Quaternion.html
        tf2::Quaternion q1{source.transform.rotation.x,
                           source.transform.rotation.y,
                           source.transform.rotation.z,
                           -source.transform.rotation.w};
        tf2::Quaternion q2{target.transform.rotation.x,
                           target.transform.rotation.y,
                           target.transform.rotation.z,
                           -source.transform.rotation.w};

        q2 *= q1;

        Eigen::Vector3d v1, v2;
        tf2::fromMsg(source.transform.translation, v1);
        tf2::fromMsg(target.transform.translation, v2);

        return std::pair<double, double>{(v2 - v1).norm(), tfDegrees(q2.getAngle())};
    }
}

PLUGINLIB_EXPORT_CLASS(icp_node::IcpNode, nodelet::Nodelet);
