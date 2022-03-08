//
// Created by 10462 on 2022/3/8.
//

#ifndef RINO_CLUSTER_H
#define RINO_CLUSTER_H

#include <vector>
#include <queue>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "config.h"

extern bool dense_cluster_expand(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                 pcl::KdTreeFLANN<pcl::PointXYZRGB> &tree,
                                 std::vector<int> &cluster_index,
                                 int point_index, int cluster_id);
#endif //RINO_CLUSTER_H
