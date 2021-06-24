//
// Created by mrs on 24/06/2021.
//

#include "tools.h"

void tools_box_filter_cloud(const pc_XYZ_t::Ptr &processing_point_cloud, float N, bool inner) {
    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(Eigen::Vector4f(-N, -N, -N, 1));
    box_filter.setMax(Eigen::Vector4f(N, N, N, 1));
    box_filter.setNegative(inner);
    box_filter.setInputCloud(processing_point_cloud);
    box_filter.filter(*processing_point_cloud);
}
