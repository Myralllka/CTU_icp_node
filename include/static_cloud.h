//
// Created by mrs on 25/06/2021.
//

#pragma once

#include <aliases.h>
#include <tools.h>
#include <string>

pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(const std::string &filename);
