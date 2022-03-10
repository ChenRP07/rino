//
// Created by 10462 on 2022/3/9.
//

#include "Octree.h"
#include <thread>
std::string filename = "../data/loot/";
int GOF = 2;
void proc(int i)
{
    Cloud pc;
    pc.set_ref_point_cloud(filename + "loot_vox10_" + std::to_string(i) + ".ply");
    pc.constant_clustering();
    std::vector<std::vector<Cloud>> subclouds(GOF - 1);
    for (int j = 1; j < GOF; j++)
    {
        pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
        pcl::io::loadPLYFile(filename + "loot_vox10_" + std::to_string(i + j) + ".ply", new_cloud);
        pc.change_point_cloud(new_cloud);
        std::cout << "read cloud " << i + j << std::endl;
        pc.centroid_alignment();
        float mse = pc.total_base_icp();
        std::cout << "total mse : " << mse << std::endl;
        pc.cluster_matching(subclouds[j - 1]);
    }
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>>> compression(subclouds[0].size());
    for (size_t j = 0; j < subclouds[0].size(); j++)
    {
        compression[j].push_back(subclouds[0][j].ref_point_cloud);
        for (size_t t = 0; t < subclouds.size(); t++)
            compression[j].push_back(subclouds[t][j].point_cloud);
    }
    std::ofstream outfile("outdata" + std::to_string(i) + ".dat", std::ios::binary | std::ios::app);
    for (size_t j = 0; j < compression.size(); j++)
    {
        Octree tree;
        tree.set_input_cloud(compression[j]);
        std::cout << "Octree " << i + j + 1<< std::endl;
        tree.compression(outfile);
        std::cout << "compress " << i + j + 1<< std::endl;
    }
    std::cout << "save " << i << std::endl;
}
int main()
{
    int start = 1000;
    std::thread ths[50];
    for (int h = 0; h < 50; h++)
        ths[h] = std::thread(proc, start + h * GOF);
    for (auto &th : ths)
        th.join();
    for (int h = 0; h < 50; h++)
        ths[h] = std::thread(proc, start + 100 + h * GOF);
    for (auto &th : ths)
        th.join();
    for (int h = 0; h < 50; h++)
        ths[h] = std::thread(proc, start + 200 + h * GOF);
    for (auto &th : ths)
        th.join();
    return 0;
}