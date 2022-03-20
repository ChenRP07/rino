//
// Created by 10462 on 2022/3/9.
//

#include "Octree.h"
#include <thread>
#include <mutex>
#include <queue>
#include <numeric>

std::queue<int> frame_pool;
std::string filename = "../data/loot/loot_vox10_";
int GOF = 3;
std::ofstream data_output("output.txt");
std::ofstream data_output1("output1.txt");
std::mutex data_output_mutex;
std::mutex pool_mutex;

float Hamming(std::vector<uint8_t> &points)
{
    int min_cnt = INT_MAX;
    for (int i = 0; i < points.size(); i++)
        if (occupation_cnt((char)points[i]) < min_cnt)
            min_cnt = occupation_cnt((char)points[i]);
    int sum_hamming = 0;
    for (int i = 0; i < points.size(); i++)
        for (int j = i + 1; j < points.size(); j++)
            sum_hamming += occupation_cnt((char)(points[i] ^ points[j]));
    data_output1 << sum_hamming << " " << min_cnt + 1 << " " << (float)(sum_hamming) / (float)(min_cnt + 1) << std::endl;
    return (float)(sum_hamming) / (float)(min_cnt + 1);
}

void proc()
{
    while (true)
    {
        int i = -1;
        bool flag = false;
        pool_mutex.lock();
        if (!frame_pool.empty())
        {
            i = frame_pool.front();
            frame_pool.pop();
            flag = true;
        }
        pool_mutex.unlock();

        if (!flag)
            return;

        Cloud pc;
        pc.set_ref_point_cloud(filename + std::to_string(i) + ".ply");
        pc.constant_clustering();
        std::cout << "load reference point cloud " << i << std::endl;

        std::vector<std::vector<Cloud>> subclouds(GOF - 1);
        std::ofstream aux_file("../test/data/aux_" + std::to_string(i) + ".dat");
        for (int j = 1; j < GOF; j++)
        {
            pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
            pcl::io::loadPLYFile(filename + std::to_string(i + j) + ".ply", new_cloud);
            pc.change_point_cloud(new_cloud);
            std::cout << "load point cloud " << i + j << std::endl;

            pc.centroid_alignment();
            float mse = pc.total_base_icp();
            std::cout << "point cloud " << i + j << " total mse " << mse << std::endl;
            std::vector<int> point_div_index;
            pc.cluster_matching(subclouds[j - 1]);
        }

        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>>> compression(subclouds[0].size());

        for (size_t j = 0; j < subclouds[0].size(); j++)
        {
            compression[j].push_back(subclouds[0][j].ref_point_cloud);
            for (size_t h = 0; h < subclouds.size(); h++)
                compression[j].push_back(subclouds[h][j].point_cloud);
        }

        /* number of clusters */
        aux_file << subclouds[0].size() << std::endl;
        //std::vector<Octree> octrees(compression.size());
        std::ofstream outfile("../test/data/outdata_" + std::to_string(i) + ".dat", std::ios::binary);
        for (size_t j = 0; j < compression.size(); j++)
        {
            Octree tree;
            pcl::PointXYZ center = tree.set_input_cloud(compression[j]);
            for (size_t h = 0; h < tree.leafs[0].size(); h++)
            {
                std::vector<uint8_t> occs;
                for (size_t k = 0; k < tree.leafs.size(); k++)
                    occs.push_back(tree.leafs[k][h]);
                //data_output1 << Hamming(occs) << std::endl;
                Hamming(occs);
            }
            int str_cnt = tree.compression(outfile);
            //pcl::PointXYZ center = octrees[j].set_input_cloud(compression[j]);
            aux_file << str_cnt << std::endl;
            aux_file << center.x << " " << center.y << " " << center.z << " " << tree.tree_range << std::endl;
            for (size_t h = 0; h < subclouds.size(); h++)
                aux_file << subclouds[h][j].transformation_matrix << std::endl;
            //std::cout<<"compress "<<j<<std::endl;
        }
//         std::vector<int> tree_size, residual_size;
//         int tree_cnt = Octree_compression(outfile, octrees, tree_size);
//         int res_cnt = residual_compression(outfile, octrees, residual_size);
//         data_output_mutex.lock();
//         data_output << i << " "
//                     << tree_cnt << " " << std::accumulate(tree_size.begin(), tree_size.end(), 0) << " "
//                     << res_cnt << " " << std::accumulate(residual_size.begin(), residual_size.end(), 0)
//                     << std::endl;
//
//
//         data_output_mutex.unlock();
    }
}
int main()
{
//    int start = 1000;
//
//    for (int i = start; i < start + 300; i += GOF)
//        frame_pool.push(i);
//    std::thread ths[50];
//    for (int h = 0; h < 50; h++)
//        ths[h] = std::thread(proc);
//    for (auto &th : ths)
//        th.join();
    frame_pool.push(1000);
    proc();

    return 0;
}