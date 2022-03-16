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

extern int occupation_cnt(char occ);
extern uint8_t occupation_table[8];
extern void occupation_pos(std::vector<int> &pos, uint8_t occ);
extern void cloud_merge(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<pcl::PointXYZRGB> &part);
class Octree
{
public:
    std::vector<std::vector<uint8_t>> tree;
    //std::vector<std::vector<int>> points;
    std::vector<std::vector<uint8_t>> leafs;

    const float min_resolution = 2.0f;
    float tree_range;

    Octree();
    explicit Octree(float range);

    int add_tree_node(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds,
                       int height, float Res, pcl::PointXYZ center);

    pcl::PointXYZ set_input_cloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds);

    int compression(std::ofstream &outfile);
    void decompression(std::ifstream &infile, int total_size, int GOF);
    void reconstruct(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds,
                     pcl::PointXYZ center, float cloud_range, float min_res);
};

extern int Octree_compression(std::ofstream &outfile, std::vector<Octree> &octrees,
                               std::vector<int> &tree_size);

extern int residual_compression(std::ofstream &outfile, std::vector<Octree> &octrees,
                                std::vector<int> &residual_size);

/*
 * 根据point和leaf的取值范围将其转化为若干位的变量，写成bit流再进行压缩。
 * */
#endif