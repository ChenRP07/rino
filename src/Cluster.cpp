//
// Created by 10462 on 2022/3/8.
//
#include "Cluster.h"

box::box(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
    this->max_x = -FLT_MAX, this->max_y = -FLT_MAX, this->max_z = -FLT_MAX;
    this->min_x = FLT_MAX, this->min_y = FLT_MAX, this->min_z = FLT_MAX;

    for (auto & i : cloud)
    {
        this->max_x = this->max_x > i.x ? this->max_x : i.x;
        this->max_y = this->max_y > i.y ? this->max_y : i.y;
        this->max_z = this->max_z > i.z ? this->max_z : i.z;

        this->min_x = this->min_x < i.x ? this->min_x : i.x;
        this->min_y = this->min_y < i.y ? this->min_y : i.y;
        this->min_z = this->min_z < i.z ? this->min_z : i.z;
    }

    this->point_cloud.resize(cloud.size());
    for (size_t i = 0; i < cloud.size(); i++)
        this->point_cloud[i] = cloud[i];
}

int box::max_range() const
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
    for (auto & i : this->point_cloud)
    {
        centroid.x += i.x;
        centroid.y += i.y;
        centroid.z += i.z;
    }
    centroid.x /= (float)this->point_cloud.size();
    centroid.y /= (float)this->point_cloud.size();
    centroid.z /= (float)this->point_cloud.size();
    return centroid;
}

void constant_cluster_centroid(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                               pcl::PointCloud<pcl::PointXYZRGB> &centroids)
{
    std::vector<box> blocks(1, box(cloud));

    int ppc = (int)cloud.size() / CONSTANT_CLUSTER_NUMBER;

    bool flag = false;
    while (!flag)
    {
        flag = true;
        std::vector<box> old_blocks;
        for (auto & block : blocks)
            old_blocks.push_back(block);
        blocks.clear();
        for (auto & old_block : old_blocks)
        {
            if ((int)old_block.point_cloud.size() > ppc)
            {
                float mid_x = (old_block.min_x + old_block.max_x) / 2;
                float mid_y = (old_block.min_y + old_block.max_y) / 2;
                float mid_z = (old_block.min_z + old_block.max_z) / 2;
                int divide_dim = old_block.max_range();
                pcl::PointCloud<pcl::PointXYZRGB> b1, b2;
                if (divide_dim == 0)
                {
                    for (auto & j : old_block.point_cloud)
                        if (j.x > mid_x)
                            b1.push_back(j);
                        else
                            b2.push_back(j);
                } else if (divide_dim == 1)
                {
                    for (auto & j : old_block.point_cloud)
                        if (j.y > mid_y)
                            b1.push_back(j);
                        else
                            b2.push_back(j);
                } else if (divide_dim == 2)
                {
                    for (auto & j : old_block.point_cloud)
                        if (j.z > mid_z)
                            b1.push_back(j);
                        else
                            b2.push_back(j);
                }
                blocks.emplace_back(b1);
                blocks.emplace_back(b2);
                flag = false;
            }
            else
                blocks.push_back(old_block);
        }
    }
    std::sort(blocks.begin(), blocks.end(),
              [](box &b1, box &b2)
              { return b1.point_cloud.size() < b2.point_cloud.size();});

    centroids.clear();
    for (int i = 0; i < CONSTANT_CLUSTER_NUMBER; i++)
        centroids.push_back(blocks[i].compute_centroid());
}
