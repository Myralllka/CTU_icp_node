//
// Created by myralllka on 29/03/2021.
//
#pragma once

#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <nodelet/nodelet.h>
#include <thread>
#include <utility>

#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <pcl/common/time.h>

namespace icp_node
{

  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;

  class IcpNode : public nodelet::Nodelet
  {
  public:
    void onInit() override;

    void callback_cloud(const PointCloud::ConstPtr& msg);

    void callback_pose(const nav_msgs::Odometry::ConstPtr& msg);

    void process_cloud(const PointCloud::ConstPtr& msg_input_cloud);

  private:
    // it's a good practice to mark member variables to easily distinguish them from local variables
    // and avoid accidentally modifying them
    /* ros::NodeHandle nh; */  // you don't need to keep the node handle after initialization

    ros::Subscriber m_sub_pc;
    ros::Subscriber m_sub_position;

    ros::Publisher m_pub_orig;
    ros::Publisher m_pub_last;
    tf2_ros::TransformBroadcaster m_tf_broadcaster;

    // aim to be consistent with naming
    /* tf2_ros::Buffer tfBuffer; */
    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_listener{m_tf_buffer};

    Eigen::Affine3d m_latest_gt_pose;
    Eigen::Affine3d m_origin_gt_pose;

    // always document what exactly is locked by a mutex (for your own good :)
    std::mutex m_processing_mutex;
    PointCloud::ConstPtr m_origin_cloud;
    PointCloud::ConstPtr m_previous_cloud;
    Eigen::Affine3d m_global_transformation = Eigen::Affine3d::Identity();

    Eigen::Affine3d pair_align(const PointCloud::ConstPtr& src, const PointCloud::ConstPtr& tgt, PointCloud& res);

    std::optional<Eigen::Affine3d>
    get_transformation_to_frame(const std::string& from_frame_id, const std::string& to_frame_id, const ros::Time& at_time);

    // consider using non-member functions instead of static methods if applicable to avoid having to define the method signature it in the header
    // also, non-member functions are usable by other objects as well (which makes sense in this case)
    /* static std::pair<double, double> compare_two_positions(const geometry_msgs::TransformStamped &source, */
    /*                                                        const geometry_msgs::TransformStamped &target); */

    // | --------------- Parameters, loaded from ROS -------------- |
  private:
    std::string m_uav_name;

    double m_icp_max_corr_dist;
    double m_icp_fitness_eps;
    double m_icp_tf_eps;
    int m_icp_max_its;
  };
}  // namespace icp_node
