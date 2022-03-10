#ifndef RINO_OCTREE_H
#define RINO_OCTREE_H

#include "Cloud.h"

struct cloud_range
{
    float max_x;
    float min_x;
    float max_y;
    float min_y;
    float max_z;
    float min_z;

    explicit cloud_range(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds);
    float max_range();
};

extern bool clouds_empty(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds);

extern pcl::PointXYZ new_center(pcl::PointXYZ &center, int pos, float res);

extern void cloud_segmentation(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &cloud,
                               std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>>> &subclouds,
                               pcl::PointXYZ center);
class Octree
{
public:
    std::vector<std::vector<uint8_t>> tree;
    std::vector<std::vector<int>> points;
    std::vector<std::vector<uint8_t>> leafs;

    float min_resolution;
    float tree_range;

    Octree();
    explicit Octree(float range);
    Octree(float res, float range);

    int add_tree_node(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds,
                       int height, float Res, pcl::PointXYZ center);

    void set_input_cloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds);

    void compression(std::ofstream &outfile);
};

/*
 * 根据point和leaf的取值范围将其转化为若干位的变量，写成bit流再进行压缩。
 * */
#endif