//
// Created by mrs on 24/06/2021.
//

#pragma once

#include "IcpNode.h"
#include "aliases.h"
#include <pcl/filters/crop_box.h>

void tools_box_filter_cloud(const pc_XYZ_t::Ptr &processing_point_cloud, float N, bool inner = false);
