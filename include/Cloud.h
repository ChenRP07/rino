/*
 * Copyright (c) Chen Ruopeng, SDU, Univ.
 * All rights reserved.
 *
 * This source code is licensed under the Mozilla Public license (found in the
 * LICENSE file in the root directory of this source tree).
 */

#ifndef RINO_CLOUD_H
#define RINO_CLOUD_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <eigen3/Eigen/Dense>

#include "Cluster.h"

class Cloud
{
public:
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;  /* point cloud */
    pcl::PointCloud<pcl::PointXYZRGB> ref_point_cloud;  /* point cloud of i-frame, i.e. reference cloud */
    Eigen::Matrix4f transformation_matrix; /* the transformation matrix from original cloud to this */
    std::vector<int> ref_cluster_index;

    Cloud();    /* constructor */
    int set_point_cloud(std::string);  /* input point cloud */
    int set_ref_point_cloud(std::string);  /* input reference cloud */

    void change_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> &point_cloud);
    void change_ref_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> &ref_point_cloud);

    void centroid_alignment();  /* translate point_cloud to align with ref_point_cloud */
    float total_base_icp();   /* transform point_cloud to align with ref_point_cloud */
    void overlap_segmentation(Cloud &, Cloud &);    /* segment both point_cloud and ref_point_cloud */

    void dense_clustering();    /* dense based clustering */
    void constant_clustering(); /* clustering point clouds into blocks
                                with approximately the same number of points */

    void cluster_matching(std::vector<Cloud> &subclouds);

    void ref_point_cloud_cluster_output(std::string address);
    
};

/* point transformation implemented by matrix multiplication */
extern void cloud_transformation(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                 Eigen::Matrix4f matrix);

extern float local_icp(pcl::PointCloud<pcl::PointXYZRGB> &ref_cluster,
                       pcl::PointCloud<pcl::PointXYZRGB> &point_cloud,
                       Cloud &cloud);

#endif