//
// Created by 10462 on 2022/3/8.
//
#include "Cluster.h"

box::box(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
    this->max_x = -FLT_MAX, this->max_y = -FLT_MAX, this->max_z = -FLT_MAX;
    this->min_x = FLT_MAX, this->min_y = FLT_MAX, this->min_z = FLT_MAX;

    for (size_t i = 0; i < cloud.size(); i++)
    {
        this->max_x = this->max_x > cloud[i].x ? this->max_x : cloud[i].x;
        this->max_y = this->max_y > cloud[i].y ? this->max_y : cloud[i].y;
        this->max_z = this->max_z > cloud[i].z ? this->max_z : cloud[i].z;

        this->min_x = this->min_x < cloud[i].x ? this->min_x : cloud[i].x;
        this->min_y = this->min_y < cloud[i].y ? this->min_y : cloud[i].y;
        this->min_z = this->min_z < cloud[i].z ? this->min_z : cloud[i].z;
    }

    this->point_cloud.resize(cloud.size());
    for (size_t i = 0; i < cloud.size(); i++)
        this->point_cloud[i] = cloud[i];
}

int box::max_range()
{
    float range_x = this->max_x - this->min_x;
    float range_y = this->max_y - this->min_y;
    float range_z = this->max_z - this->min_z;
    if (range_x >= range_y && range_x >= range_z)
        return 0;
    else if (range_y >= range_x && range_y >= range_z)
        return 1;
    else if (range_z >= range_x && range_z >= range_y)
        return 2;
    return -1;
}

pcl::PointXYZRGB box::compute_centroid()
{
    pcl::PointXYZRGB centroid(0.0f, 0.0f, 0.0f);
    for (size_t i = 0; i < this->point_cloud.size(); i++)
    {
        centroid.x += this->point_cloud[i].x;
        centroid.y += this->point_cloud[i].y;
        centroid.z += this->point_cloud[i].z;
    }
    centroid.x /= this->point_cloud.size();
    centroid.y /= this->point_cloud.size();
    centroid.z /= this->point_cloud.size();
    return centroid;
}

bool dense_cluster_expand(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                          pcl::KdTreeFLANN<pcl::PointXYZRGB> &tree,
                          std::vector<int> &cluster_index,
                          int point_index, int cluster_id)
{
    /* do nearest neighbor search in cloud and generate result */
    std::vector<int> seed_index;
    std::vector<float> seed_distance;
    tree.radiusSearch(cloud[point_index], DENSE_CLUSTERING_NEIGHBOR_RADIUS, seed_index, seed_distance);

    /* noise point */
    if (seed_index.size() <= DENSE_CLUSTERING_NEIGHBOR_NUMBER)
    {
        cluster_index[point_index] = 0;
        return false;
    }
    else /* core point */
    {
        std::queue<int> seeds;  /* seed points */
        for (size_t i = 0; i < seed_index.size(); i++)
        {
            /* add neighbors to cluster */
            cluster_index[seed_index[i]] = cluster_id;
            /* this point itself won't be processed next iteration */
            if (seed_index[i] != point_index)
                seeds.push(seed_index[i]);
        }

        /* for all neighbors */
        while (!seeds.empty())
        {
            /* do nearest neighbors search */
            int current_point = seeds.front();
            std::vector<int> result_index;
            std::vector<float> result_distance;
            tree.radiusSearch(cloud[current_point], DENSE_CLUSTERING_NEIGHBOR_RADIUS, result_index, result_distance);

            /* core point, need to be processed next iteration */
            if (result_index.size() > DENSE_CLUSTERING_NEIGHBOR_NUMBER)
            {
                for (size_t h = 0; h < result_index.size(); h++)
                {
                    /* all points without other cluster */
                    if (cluster_index[result_index[h]] == 0 || cluster_index[result_index[h]] == -1)
                    {
                        /* unclassified point */
                        if (cluster_index[result_index[h]] == -1)
                            seeds.push(result_index[h]);
                        cluster_index[result_index[h]] = cluster_id;
                    }
                }
            }
            seeds.pop();
        }
        return true;
    }
}

void constant_cluster_centroid(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                               pcl::PointCloud<pcl::PointXYZRGB> &centroids)
{
    std::vector<box> blocks(1, box(cloud));

    int ppc = cloud.size() / CONSTANT_CLUSTER_NUMBER;

    bool flag = false;
    while (!flag)
    {
        flag = true;
        std::vector<box> old_blocks;
        for (size_t i = 0; i < blocks.size(); i++)
            old_blocks.push_back(blocks[i]);
        blocks.clear();
        for (size_t i = 0; i < old_blocks.size(); i++)
        {
            if (old_blocks[i].point_cloud.size() > ppc)
            {
                float mid_x = (old_blocks[i].min_x + old_blocks[i].max_x) / 2;
                float mid_y = (old_blocks[i].min_y + old_blocks[i].max_y) / 2;
                float mid_z = (old_blocks[i].min_z + old_blocks[i].max_z) / 2;
                int divide_dim = old_blocks[i].max_range();
                pcl::PointCloud<pcl::PointXYZRGB> b1, b2;
                if (divide_dim == 0)
                {
                    for (size_t j = 0; j < old_blocks[i].point_cloud.size(); j++)
                        if (old_blocks[i].point_cloud[j].x > mid_x)
                            b1.push_back(old_blocks[i].point_cloud[j]);
                        else
                            b2.push_back(old_blocks[i].point_cloud[j]);
                } else if (divide_dim == 1)
                {
                    for (size_t j = 0; j < old_blocks[i].point_cloud.size(); j++)
                        if (old_blocks[i].point_cloud[j].y > mid_y)
                            b1.push_back(old_blocks[i].point_cloud[j]);
                        else
                            b2.push_back(old_blocks[i].point_cloud[j]);
                } else if (divide_dim == 2)
                {
                    for (size_t j = 0; j < old_blocks[i].point_cloud.size(); j++)
                        if (old_blocks[i].point_cloud[j].z > mid_z)
                            b1.push_back(old_blocks[i].point_cloud[j]);
                        else
                            b2.push_back(old_blocks[i].point_cloud[j]);
                }
                blocks.push_back(box(b1));
                blocks.push_back(box(b2));
                flag = false;
            }
            else
                blocks.push_back(old_blocks[i]);
        }
    }
    std::sort(blocks.begin(), blocks.end(),
              [](box &b1, box &b2)
              { return b1.point_cloud.size() < b2.point_cloud.size();});

    centroids.clear();
    for (int i = 0; i < CONSTANT_CLUSTER_NUMBER; i++)
        centroids.push_back(blocks[i].compute_centroid());
}
