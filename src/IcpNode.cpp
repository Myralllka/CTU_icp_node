#include "IcpNode.h"
#include <boost/smart_ptr/make_shared_array.hpp>
#include <mrs_lib/param_loader.h>
#include <filesystem>
#include "tools.h"
// Matous code https://mrs.felk.cvut.cz/gitlab/vrbamato/uav_detect/blob/ouster/src/pcl_selfloc_nodelet.cpp#L420

namespace icp_node {
    void IcpNode::callback_cloud(const pc_XYZ_t::ConstPtr &msg) {
        if (!m_apriori_map_initialized) {
            NODELET_WARN_THROTTLE(1.0, "[PCLSelfLocalizator]: Waiting for intialization of the apriori static map.");
            return;
        }
        pcl::ScopeTime t1("callback");
        pc_XYZ_t::Ptr input_pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*msg.get(), *input_pt_cloud);
        tools_box_filter_cloud(input_pt_cloud, 0.4, true);
        tools_box_filter_cloud(input_pt_cloud, 25, false);

        // I don't see why the input filtration should be split between this method and the `processCloud` method
        pcl::VoxelGrid<p_XYZ_t> voxel_filter;
        voxel_filter.setLeafSize(0.5, 0.5, 0.5);
        voxel_filter.setInputCloud(input_pt_cloud);
        voxel_filter.filter(*input_pt_cloud);

        process_cloud(input_pt_cloud);
        m_pub_static_pc.publish(m_static_cloud);
    }

    // in robotics, "position" typically only means a point in space, whereas "pose" includes orientation
    std::pair<float, float> compare_two_poses(const Eigen::Affine3f &source, const Eigen::Affine3f &target) {
        // try to make as variables `const` wherever applicable (look up "c++ const-correctness")
        const float translation_diff = (source.translation() - target.translation()).norm();
        const float angle_diff = Eigen::AngleAxisf().fromRotationMatrix(
                source.rotation().inverse() * target.rotation()).angle();
        return {translation_diff, angle_diff};
    }

    void IcpNode::process_cloud(const pc_XYZ_t::ConstPtr &msg_input_cloud) {
        std::lock_guard<std::mutex> lock(m_processing_mutex);
        if (m_origin_cloud == nullptr) {
            m_origin_cloud = msg_input_cloud;
            m_previous_cloud = msg_input_cloud;
            m_origin_gt_pose = m_latest_gt_pose;
            // why do you modify the frame_id? what frame is "world/local_origin"?
//            origin_position.child_frame_id = "world/local_origin";
        } else {
            // Find the transformation to align the current message to the previous cloud
            auto aligned_input_cloud = boost::make_shared<pc_XYZ_t>();
            Eigen::Affine3d align_transformation = pair_align(msg_input_cloud, m_previous_cloud, *aligned_input_cloud);
            // update the estimated global TF
            m_global_transformation = align_transformation * m_global_transformation;

            // Publish the latest tf2 transformation
            {
                // Use blocks (curly braces) to control scope of local variables and avoid accidentally using them further down.
                // Also, when you do something like this, consider moving it to a separate function.
                geometry_msgs::TransformStamped msg;
                pcl_conversions::fromPCL(msg_input_cloud->header.stamp, msg.header.stamp);
                msg.header.frame_id = m_uav_name + "/local_origin";
                msg.child_frame_id = m_uav_name + "/icp_origin";
                msg.transform = tf2::eigenToTransform(
                        static_cast<const Eigen::Affine3d>(m_global_transformation.inverse())).transform;
                m_tf_broadcaster.sendTransform(msg);
            }

            // publish some debug aligned pointclouds if requested
            if (m_pub_last.getNumSubscribers() > 0) {
                aligned_input_cloud->header.stamp = msg_input_cloud->header.stamp;
                m_pub_last.publish(aligned_input_cloud);
            }

            if (m_pub_orig.getNumSubscribers() > 0) {
                pc_XYZ_t::Ptr tfd_orig_pc = boost::make_shared<pc_XYZ_t>();
                pcl::transformPointCloud(*m_origin_cloud, *tfd_orig_pc, m_global_transformation.inverse());
                tfd_orig_pc->header.stamp = msg_input_cloud->header.stamp;
                m_pub_orig.publish(tfd_orig_pc);
            }

            // ground-truth transformation from origin (first received pose) to the current UAV pose
            const Eigen::Affine3d orig2cur_tf_gt = m_origin_gt_pose.inverse() * m_latest_gt_pose;
            const auto epsilon = compare_two_poses(static_cast<const Eigen::Affine3f>(m_global_transformation),
                                                   static_cast<const Eigen::Affine3f>(orig2cur_tf_gt));

//            std::cout << "GT translation:  " << orig2cur_tf_gt.translation().transpose() << "\n";
//            std::cout << "ICP translation: " << m_global_transformation.translation().transpose() << "\n";
            std::cout << "\ttransformation error is: " << epsilon.first << "m.\n"
                      << "\trotation error is: " << epsilon.second / M_PI * 180.0 << "deg.\n";

            // update the previous cloud message
            m_previous_cloud = msg_input_cloud;
        }
    }

    void IcpNode::onInit() {
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

        mrs_lib::ParamLoader pl(nh);
        std::string static_cloud_filename;
        NODELET_INFO("Loading static parameters:");
        pl.loadParam("uav_name", m_uav_name);
        pl.loadParam("map_frame_id", m_map_frame_id);
        pl.loadParam("icp/max_correspondence_distance", m_icp_max_corr_dist);
        pl.loadParam("icp/euclidean_fitness_epsilon", m_icp_fitness_eps);
        pl.loadParam("icp/transformation_epsilon", m_icp_tf_eps);
        pl.loadParam("icp/maximum_iterations", m_icp_max_its);
        pl.loadParam("static_cloud_filename", static_cloud_filename);
//        auto static_cloud_filename = pl.loadParam2<std::string>("icp/static_cloud_filename");

        // handle missing parameters
        if (!pl.loadedSuccessfully()) {
            NODELET_ERROR("Some compulsory parameters were not loaded successfully, ending the node.");
            ros::requestShutdown();
            return;
        }

        m_pub_orig = nh.advertise<pc_XYZ_t>("aligned_orig", 1);
        m_pub_last = nh.advertise<pc_XYZ_t>("aligned_prev", 1);
        m_pub_static_pc = nh.advertise<sensor_msgs::PointCloud2>("static_pc", 1, true);
        m_sub_pc = nh.subscribe("points_in", 1, &IcpNode::callback_cloud, this);
        m_sub_position = nh.subscribe("ground_truth", 1, &IcpNode::callback_pose, this);
        m_apriori_map_initialized = false;

        initialize_static_map(static_cloud_filename);
    }

    Eigen::Affine3d
    IcpNode::pair_align(const pc_XYZ_t::ConstPtr &src,
                        const pc_XYZ_t::ConstPtr &tgt,
                        pc_XYZ_t &res) {
        // the clouds are already filtered - this is redundant
        pcl::IterativeClosestPointNonLinear<p_XYZ_t, p_XYZ_t> icp;

        // parametrize stuff like this
        icp.setMaxCorrespondenceDistance(m_icp_max_corr_dist);
        icp.setEuclideanFitnessEpsilon(m_icp_fitness_eps);
        icp.setTransformationEpsilon(m_icp_tf_eps);
        icp.setMaximumIterations(m_icp_max_its);

        // Align
        icp.setInputSource(src);
        icp.setInputTarget(tgt);

        icp.align(res);

        return Eigen::Affine3d(icp.getFinalTransformation().cast<double>());
    }

    void IcpNode::initialize_static_map(const std::string &filename) {
        const pc_XYZ_t::Ptr loaded_cloud = load_cloud(filename);
        if (loaded_cloud == nullptr) {
            NODELET_ERROR("Failed to load the static pointcloud! Ending the node.");
            ros::shutdown();
            return;
        } else {
            m_static_cloud = loaded_cloud;
            NODELET_INFO("Loaded a static cloud with %lu points.", m_static_cloud->size());

            pc_XYZ_t::Ptr tmp_cloud = boost::make_shared<pc_XYZ_t>();
            pcl::VoxelGrid<p_XYZ_t> vg;
            vg.setInputCloud(m_static_cloud);
            vg.setLeafSize(m_apriori_map_voxelgrid_leaf_size, m_apriori_map_voxelgrid_leaf_size,
                           m_apriori_map_voxelgrid_leaf_size);
            vg.filter(*tmp_cloud);
            std::swap(m_static_cloud, tmp_cloud);
            NODELET_INFO("Downsampled the static cloud to %lu points.", m_static_cloud->size());

            const Eigen::Vector3f translation =
                    Eigen::Vector3f(m_apriori_map_tf_x, m_apriori_map_tf_y, m_apriori_map_tf_z) +
                    Eigen::Vector3f(m_apriori_map_correction_x, m_apriori_map_correction_y, m_apriori_map_correction_z);
            const Eigen::Matrix3f rotation = Eigen::AngleAxisf(static_cast<float>(m_apriori_map_tf_yaw / 180.0 * M_PI),
                                                               Eigen::Vector3f::UnitZ()).toRotationMatrix();
            Eigen::Affine3f corr_tf = Eigen::Affine3f::Identity();
            corr_tf.rotate(rotation);
            corr_tf.translate(translation);

            pc_XYZ_t::Ptr tfd_static_cloud = boost::make_shared<pc_XYZ_t>();      // contains points, which are apriori known to be a part of the static background
            pcl::transformPointCloud(*m_static_cloud, *tfd_static_cloud, corr_tf);
            tfd_static_cloud->header.frame_id = m_map_frame_id;

            const ros::Time stamp = ros::Time::now();
            pcl_conversions::toPCL(stamp, tfd_static_cloud->header.stamp);
            m_pub_static_pc.publish(m_static_cloud);
        }
        m_apriori_map_initialized = true;
    }

    // in robotics, "position" typically only means a point in space, whereas "pose" includes orientation
    void IcpNode::callback_pose(const nav_msgs::Odometry::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(m_processing_mutex);
        tf2::fromMsg(msg->pose.pose, m_latest_gt_pose);
    }

}  // namespace icp_node

pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(const std::string &filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::ifstream fs;
    std::cout << std::filesystem::current_path() << std::endl;
    fs.open(filename.c_str(), std::ios::binary);
    if (!fs.is_open() || fs.fail()) {
        PCL_ERROR("Could not open file '%s'! Error : %s\n", filename.c_str(), strerror(errno));
        fs.close();
        return nullptr;
    }

    while (!fs.eof()) {
        std::string line;
        std::getline(fs, line);
        // Ignore empty lines
        if (line.empty())
            continue;

        // Tokenize the line
        boost::trim(line);
        /* const std::vector<std::string> st = split(line, ' '); */
        std::vector<std::string> st;
        boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

        if (st.size() != 3) {
            PCL_WARN("Read line with a wrong number of elements: %d (expected 3). The line: '%s'.", (int) st.size(),
                     line.c_str());
            continue;
        }

        cloud->push_back(pcl::PointXYZ(
                float(atof(st[0].c_str())),
                float(atof(st[1].c_str())),
                float(atof(st[2].c_str()))
        ));

        if (cloud->size() % 100000 == 0)
            PCL_INFO("Loaded %lu points so far.\r\n", cloud->size());
    }
    fs.close();

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

PLUGINLIB_EXPORT_CLASS(icp_node::IcpNode, nodelet::Nodelet);
