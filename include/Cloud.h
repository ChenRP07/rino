/*
 * Copyright (c) Chen Ruopeng, SDU, Univ.
 * All rights reserved.
 *
 * This source code is licensed under the Mozilla Public license (found in the
 * LICENSE file in the root directory of this source tree).
 */
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>

class Cloud
{
public:
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> ref_point_cloud;
    Eigen::Matrix4f translation_matrix;

    Cloud();
    void set_point_cloud(std::string);
    void set_ref_point_cloud(std::string);

    
}