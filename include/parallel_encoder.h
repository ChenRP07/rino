//
// Created by 10462 on 2022/3/22.
//

#ifndef RINO_PARALLEL_ENCODER_H
#define RINO_PARALLEL_ENCODER_H

#include "Octree.h"
#include <sys/types.h>
#include <ctime>
#include <utility>
#include <sys/time.h>
#include <unistd.h>

class parallel_encoder
{
public:
    Cloud point_clouds;
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>>> clusters;
    std::vector<Eigen::Matrix4f> trans;
    std::vector<uint8_t> colors;
    struct timeval time1, time2;

    void set_ref_point_cloud(const std::string& filename);
    void set_point_cloud(const std::string& filename);
    void clusters_generating();
    void clusters_matching();
    void clusters_compression(const std::string& aux_name, const std::string& geo_name, std::string color_name);
    void encoder(const std::string& reffilename, const std::string& filename, const std::string& aux_name,
                 const std::string& geo_name, std::string color_name);
};
#endif //RINO_PARALLEL_ENCODER_H
