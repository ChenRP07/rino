//
// Created by 10462 on 2022/3/14.
//
#include "Octree.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>

int GOF = 4;
void rounding(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
    for (int i = 0; i < cloud.size(); i++)
    {
        cloud[i].x = std::round(cloud[i].x);
        cloud[i].y = std::round(cloud[i].y);
        cloud[i].z = std::round(cloud[i].z);
    }
}
int main()
{
    for (int i = 1000; i < 1000 + 300; i += GOF)
    {
        std::ifstream infile("../test/data/outdata_" + std::to_string(i) + ".dat");
        std::ifstream aux_file("../test/data/aux_" + std::to_string(i) + ".dat");
        int clusters_cnt;
        aux_file >> clusters_cnt;
        std::vector<int> str_cnt(clusters_cnt);
        std::vector<float> tree_range(clusters_cnt);
        std::vector<pcl::PointXYZ> centers(clusters_cnt);
        std::vector<std::vector<Eigen::Matrix4f>> trans(clusters_cnt);
        for (int j = 0; j < str_cnt.size(); j++)
        {
            aux_file >> str_cnt[j];
            aux_file >> centers[j].x >> centers[j].y >> centers[j].z;
            aux_file >> tree_range[j];
            for (int h = 0; h < GOF - 1; h++)
            {
                Eigen::Matrix4f ma;
                for (int row = 0; row < 4; row++)
                    for (int col = 0; col < 4; col++)
                        aux_file >> ma(row, col);
                trans[j].push_back(ma);
            }
        }
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>> point_clouds(GOF);
        timeval tim1, tim2;
        gettimeofday(&tim1, NULL);
        for (int j = 0; j < str_cnt.size(); j++)
        {
            Octree tree;
            tree.decompression(infile, str_cnt[j], GOF);
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>> tree_clouds;
            tree.reconstruct(tree_clouds, centers[j], tree_range[j], 4.0f);
            for (int h = 1; h < GOF; h++)
                cloud_transformation(tree_clouds[h], trans[j][h - 1].inverse());
            for (int h = 0; h < GOF; h++)
                cloud_merge(point_clouds[h], tree_clouds[h]);
        }
        gettimeofday(&tim2, NULL);
        std::cout << "Reconstruction complete, using " << std::fixed << std::setprecision(3) << tim2.tv_sec - tim1.tv_sec + (float)(tim2.tv_usec - tim1.tv_usec) / 1000000 << "s." << std::endl;
        for (int j = 0; j < point_clouds.size(); j++)
        {
            //rounding(point_clouds[j]);
            pcl::io::savePLYFile("../test/cloud/loot_vox10_" + std::to_string(i + j) + ".ply", point_clouds[j]);
        }
    }
}
