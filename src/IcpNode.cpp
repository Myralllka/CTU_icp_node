#include <IcpNode.h>
#include <boost/smart_ptr/make_shared_array.hpp>
#include <mrs_lib/param_loader.h>
// Matous code https://mrs.felk.cvut.cz/gitlab/vrbamato/uav_detect/blob/ouster/src/pcl_selfloc_nodelet.cpp#L420
//

namespace icp_node
{
  void IcpNode::callback_cloud(const PointCloud::ConstPtr& msg)
  {
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

    // I don't see why the input filtration should be split between this method and the `processCloud` method
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setLeafSize(0.5, 0.5, 0.5);
    voxel_filter.setInputCloud(input_pt_cloud);
    voxel_filter.filter(*input_pt_cloud);

    process_cloud(input_pt_cloud);
  }

  // in robotics, "position" typically only means a point in space, whereas "pose" includes orientation
  /* std::pair<double, double> IcpNode::compare_two_positions(const Eigen::Affine3d& source, const Eigen::Affine3d& target) */
  std::pair<double, double> compare_two_poses(const Eigen::Affine3d& source, const Eigen::Affine3d& target)
  {
    // try to make as variables `const` wherever applicable (look up "c++ const-correctness")
    const double translation_diff = (source.translation() - target.translation()).norm();
    const double angle_diff = Eigen::AngleAxisd().fromRotationMatrix(source.rotation().inverse() * target.rotation()).angle();
    return {translation_diff, angle_diff};
  }

  void IcpNode::process_cloud(const PointCloud::ConstPtr& msg_input_cloud)
  {
    std::lock_guard<std::mutex> lock(m_processing_mutex);
    if (m_origin_cloud == nullptr)
    {
      m_origin_cloud = msg_input_cloud;
      m_previous_cloud = msg_input_cloud;
      m_origin_gt_pose = m_latest_gt_pose;
      // why do you modify the frame_id? what frame is "world/local_origin"?
      /* origin_position.child_frame_id = "world/local_origin"; */
    } else
    {
      // Find the transformation to align the current message to the previous cloud
      PointCloud::Ptr aligned_input_cloud = boost::make_shared<PointCloud>();
      // Avoid using names such as `tmp...` etc., use descriptive names whenever possible.
      const Eigen::Affine3d align_transformation = pair_align(msg_input_cloud, m_previous_cloud, *aligned_input_cloud);
      // update the estimated global TF
      m_global_transformation = align_transformation * m_global_transformation;

      // Publish the latest tf2 transformation
      {
        // Use blocks (curly braces) to control scope of local variables and avoid accidentaly using them further down.
        // Also, when you do something like this, consider moving it to a separate function.
        geometry_msgs::TransformStamped msg;
        pcl_conversions::fromPCL(msg_input_cloud->header.stamp, msg.header.stamp);
        // *always* consider that your code can run on ANY drone under ANY namespace
        /* msg.header.frame_id = "uav1/local_origin"; */
        /* msg.child_frame_id = "uav1/icp_estimated_origin"; */
        msg.header.frame_id = m_uav_name + "/fcu";
        msg.child_frame_id = m_uav_name + "/icp_origin";
        msg.transform = tf2::eigenToTransform(m_global_transformation.inverse()).transform;
        m_tf_broadcaster.sendTransform(msg);
      }

      // publish some debug aligned pointclouds if requested
      if (m_pub_last.getNumSubscribers() > 0)
      {
        aligned_input_cloud->header.stamp = msg_input_cloud->header.stamp;
        m_pub_last.publish(aligned_input_cloud);
      }

      if (m_pub_orig.getNumSubscribers() > 0)
      {
        // Avoid `new`! You *rarely* need it. `make_shared` or `make_unique` is exception-safe, use that instead.
        // Also, avoid using names such as `tmp...` etc., use descriptive names whenever possible.
        /* PointCloud::Ptr tmp_pc(new PointCloud); */
        PointCloud::Ptr tfd_orig_pc = boost::make_shared<PointCloud>();
        pcl::transformPointCloud(*m_origin_cloud, *tfd_orig_pc, m_global_transformation.inverse());
        tfd_orig_pc->header.stamp = msg_input_cloud->header.stamp;
        m_pub_orig.publish(tfd_orig_pc);
      }

      // ground-truth transformation from origin (first received pose) to the current UAV pose
      const Eigen::Affine3d orig2cur_tf_gt = m_origin_gt_pose.inverse()*m_latest_gt_pose;
      const auto epsilon = compare_two_poses(m_global_transformation, orig2cur_tf_gt);

      std::cout << "GT translation:  " << orig2cur_tf_gt.translation().transpose() << "\n";
      std::cout << "ICP translation: " << m_global_transformation.translation().transpose() << "\n";
      std::cout << "\ttransformation error is: " << epsilon.first << "m.\n"
                << "\trotation error is: " << epsilon.second/M_PI*180.0 << "deg.\n";

      // update the previous cloud message
      m_previous_cloud = msg_input_cloud;
    }
  }

  void IcpNode::onInit()
  {
    // this is how you should get a node handle in a nodelet
    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

    // I recommend using the mrs_lib::ParamLoader for easier parameter loading
    mrs_lib::ParamLoader pl(nh);
    NODELET_INFO("Loading static parameters:");
    pl.loadParam("uav_name", m_uav_name);

    pl.loadParam("icp/max_correspondence_distance", m_icp_max_corr_dist);
    pl.loadParam("icp/euclidean_fitness_epsilon", m_icp_fitness_eps);
    pl.loadParam("icp/transformation_epsilon", m_icp_tf_eps);
    pl.loadParam("icp/maximum_iterations", m_icp_max_its);

    // handle missing parameters
    if (!pl.loadedSuccessfully())
    {
      NODELET_ERROR("Some compulsory parameters were not loaded successfully, ending the node.");
      ros::requestShutdown();
      return;
    }

    // do not specify the namespace here - instead, namespace the whole program in the launchfile
    // (if you specify it here, what will you do when you need to run the code on a different drone?)
    /* pub = nh.advertise<PointCloud>("/uav1/points_icp/res", 1); */
    m_pub_orig = nh.advertise<PointCloud>("aligned_orig", 1);
    m_pub_last = nh.advertise<PointCloud>("aligned_prev", 1);
    // Do not specify the exact name of the topic here - instead, use a descriptive name here
    // and remap it to the exact topic in the launchfile. This way, it will be trivial to change
    // a topic name when you e.g. want to get the points from a different sensor.
    /* m_sub_pc = nh.subscribe("/uav1/os_cloud_nodelet/points", 1, &IcpNode::callback_cloud, this); */
    m_sub_pc = nh.subscribe("points_in", 1, &IcpNode::callback_cloud, this);
    m_sub_position = nh.subscribe("ground_truth", 1, &IcpNode::callback_pose, this);
  }

  // Return the main output prefferably by return value.
  // Mark output or inout parameters of a function by making them non-const reference.
  // Also, watch out for const-correctness to properly document which variables are expected to be modified
  // and which are not.
  /* void IcpNode::pair_align(const PointCloud::Ptr& src, const PointCloud::Ptr& tgt, const PointCloud::Ptr& res, Eigen::Affine3f& final_transform) */
  Eigen::Affine3d IcpNode::pair_align(const PointCloud::ConstPtr& src, const PointCloud::ConstPtr& tgt, PointCloud& res)
  {
    // the clouds are already filtered - this is redundant
    /* pcl::VoxelGrid<PointT> voxel_filter; */
    /* voxel_filter.setLeafSize(0.5, 0.5, 0.5); */

    /* voxel_filter.setInputCloud(tgt); */
    /* voxel_filter.filter(*tgt); */
    pcl::IterativeClosestPointNonLinear<PointT, PointT> icp;

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

  // in robotics, "position" typically only means a point in space, whereas "pose" includes orientation
  /* void IcpNode::callback_position(const nav_msgs::Odometry::ConstPtr& msg) */
  void IcpNode::callback_pose(const nav_msgs::Odometry::ConstPtr& msg)
  {
    /* std::lock_guard<std::mutex> lock(processing_mutex); */
    /* current_position.header = msg->header; */
    /* current_position.child_frame_id = msg->child_frame_id; */
    /* current_position.transform.translation.x = msg->pose.pose.position.x; */
    /* current_position.transform.translation.y = msg->pose.pose.position.y; */
    /* current_position.transform.translation.z = msg->pose.pose.position.z; */
    /* current_position.transform.rotation = msg->pose.pose.orientation; */
    // a more elegant solution perhaps?
    std::lock_guard<std::mutex> lock(m_processing_mutex);
    tf2::fromMsg(msg->pose.pose, m_latest_gt_pose);
  }

  // http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29
  // Again, prefer to return the main output using a return value.
  // Also, in cases like this, you should indicate whether the result is valid or not.
  std::optional<Eigen::Affine3d>
  IcpNode::get_transformation_to_frame(const std::string& from_frame_id, const std::string& to_frame_id, const ros::Time& at_time)
  {
    try
    {
      // I recommend parametrising the timeout
      const ros::Duration timeout(1.0 / 100.0);
      // Obtain transform from sensor into world frame
      const geometry_msgs::TransformStamped tf = m_tf_buffer.lookupTransform(to_frame_id, from_frame_id, at_time, timeout);
      return tf2::transformToEigen(tf.transform);
    }
    catch (tf2::TransformException& ex)
    {
      NODELET_WARN_THROTTLE(1.0, "[%s]: Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", "icp_node", from_frame_id.c_str(),
                            to_frame_id.c_str(), ex.what());
      return std::nullopt;
    }
  }
}  // namespace icp_node

PLUGINLIB_EXPORT_CLASS(icp_node::IcpNode, nodelet::Nodelet);
