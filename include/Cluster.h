//
// Created by 10462 on 2022/3/8.
//

#ifndef RINO_CLUSTER_H
#define RINO_CLUSTER_H

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

extern bool dense_cluster_expand(pcl::PointCloud<pcl::PointXYZRGB> &,
                                 pcl::KdTreeFLANN<pcl::PointXYZRGB> &,
                                 std::vector<int> &, int, int, int, float);
#endif //RINO_CLUSTER_H
