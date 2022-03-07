/*
 * Copyright (c) Chen Ruopeng, SDU, Univ.
 * All rights reserved.
 *
 * This source code is licensed under the Mozilla Public license (found in the
 * LICENSE file in the root directory of this source tree).
 */
#include <string>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <eigen3/Eigen/Dense>

#include "config.h"

class Cloud
{
public:
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;  /* point cloud */
    pcl::PointCloud<pcl::PointXYZRGB> ref_point_cloud;  /* point cloud of i-frame, i.e. reference cloud */
    Eigen::Matrix4f transformation_matrix; /* the transformation matrix from original cloud to this */

    Cloud();    /* constructor */
    int set_point_cloud(std::string);  /* input point cloud */
    int set_ref_point_cloud(std::string);  /* input reference cloud */

    void centroid_alignment();
    int total_base_icp();
    void overlap_segmentation(Cloud &, Cloud &);
}
