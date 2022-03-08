//
// Created by 10462 on 2022/3/8.
//
#include "Cluster.h"

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

