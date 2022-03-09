//
// Created by 10462 on 2022/3/8.
//

#ifndef RINO_CLUSTER_H
#define RINO_CLUSTER_H

#include <vector>
#include <queue>
#include <float.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include "config.h"

class box
{
public:
    float max_x;
    float min_x;
    float max_y;
    float min_y;
    float max_z;
    float min_z;
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;

    box(pcl::PointCloud<pcl::PointXYZRGB> &cloud);  /* initialization */
    int max_range();    /* identify the dimension that spans the greatest distance */
    pcl::PointXYZRGB compute_centroid();
};

extern bool dense_cluster_expand(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                 pcl::KdTreeFLANN<pcl::PointXYZRGB> &tree,
                                 std::vector<int> &cluster_index,
                                 int point_index, int cluster_id);

extern void constant_cluster_centroid(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                      pcl::PointCloud<pcl::PointXYZRGB> &centroids);

#endif //RINO_CLUSTER_H
