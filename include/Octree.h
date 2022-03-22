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
    float max_range() const;
};

/* color for octree */
struct point_color
{
    int node_pos;
    uint8_t red;
    uint8_t blue;
    uint8_t green;
    point_color(pcl::PointXYZRGB point, int pos);
    point_color(const pcl::PointCloud<pcl::PointXYZRGB>& points, int pos);
    point_color(const point_color& point);
    point_color& operator = (const point_color& point);
    bool operator < (const point_color& point) const;
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
extern pcl::PointXYZRGB new_point(pcl::PointXYZ &center, int pos, uint8_t red, uint8_t green, uint8_t blue);
extern float average_hamming_distance(uint8_t occ_i, uint8_t occ_p);
extern void merge_color(std::vector<point_color> &color_i, std::vector<point_color> &color_p);
class Octree
{
public:
    std::vector<std::vector<uint8_t>> tree;
    //std::vector<std::vector<int>> points;
    std::vector<std::vector<uint8_t>> leafs;
    std::vector<std::vector<point_color>> colors_i;
    std::vector<std::vector<point_color>> colors_p;

    const float min_resolution = 2.0f;
    float tree_range;

    Octree();
    explicit Octree(float range);

    int add_tree_node(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds,
                       int height, float Res, pcl::PointXYZ center);

    pcl::PointXYZ set_input_cloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds);

    int compression(std::ofstream &outfile);
    void decompression(std::string &all_data_result);
    void reconstruct(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds,
                     pcl::PointXYZ center, float cloud_range,
                     std::vector<uint8_t> &colors, int color_index);

    void color_compression(std::vector<uint8_t> &all_colors);
};


/*
 * 根据point和leaf的取值范围将其转化为若干位的变量，写成bit流再进行压缩。
 * */
#endif