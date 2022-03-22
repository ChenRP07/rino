//
// Created by 10462 on 2022/3/21.
//

#ifndef RINO_PARALLEL_DECODER_H
#define RINO_PARALLEL_DECODER_H

#include "Octree.h"
#include <queue>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <ctime>
#include <utility>

struct merge_task
{
    int index{};
    int start_index{};
    int length{};
    merge_task();
    merge_task(int pm1, int pm2, int pm3);
    merge_task(const merge_task &task);
    merge_task& operator = (const merge_task &task);
};

class parallel_decoder
{
public:
    std::vector<std::string> geometry_data_pool;    /* container for compressed string */
    std::queue<int> geometry_task_pool; /* task_pool for clusters */
    std::queue<int> reconstruction_task_pool;
    std::queue<merge_task> icloud_merge_task_pool;
    std::queue<merge_task> pcloud_merge_task_pool;
    std::vector<int> str_cnt;   /* compressed string length */
    std::vector<float> tree_range;  /* tree ranges */
    std::vector<pcl::PointXYZ> centers; /* cluster centers */
    std::vector<Eigen::Matrix4f> trans; /* cluster transformation matrix */
    int clusters_cnt;   /* clusters count */
    struct timeval time1, time2;    /* timer */
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> point_clouds;    /* point clouds */
    std::vector<Octree> trees;  /* octrees */
    std::vector<uint8_t> color_info;    /* all color information */
    std::vector<int> all_colors;
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>>> point_clusters;
    std::mutex geometry_task_pool_mutex;
    std::mutex reconstruction_task_pool_mutex;
    std::mutex cloud_merge_mutex;

    void resize_containers();   /* allocate memory for members */
    void set_aux_information(const std::string& aux_file); /* read auxiliary information from aux_file */
    void set_geometry_information(const std::string& geo_file); /* read geometry information from geo_file */
    void octree_geometry_construction();    /* thread working function */
    void parallel_octree_geometry_construction();   /* parallel octree geometry decompression */
    void set_color_information(std::string color_file);
    void octree_cloud_reconstruction();
    void parallel_cloud_reconstruction();
    void point_cloud_merge();
    void parallel_cloud_merge();
    void save_point_cloud(const std::string& cloud_name1, const std::string& cloud_name2);
    void decoder(const std::string& aux_file, const std::string& geo_file, std::string color_file,
                 const std::string& cloud_name1, const std::string& cloud_name2);
};

#endif //RINO_PARALLEL_DECODER_H
