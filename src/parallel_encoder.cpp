//
// Created by 10462 on 2022/3/22.
//

#include "parallel_encoder.h"

#include <utility>

void parallel_encoder::set_ref_point_cloud(const std::string& filename)
{
    this->point_clouds.set_ref_point_cloud(filename);
    std::cout << "Read reference point cloud : " << filename << std::endl;
}

void parallel_encoder::set_point_cloud(const std::string& filename)
{
    this->point_clouds.set_point_cloud(filename);
    std::cout << "Read point cloud : " << filename << std::endl;
}

void parallel_encoder::clusters_generating()
{
    this->point_clouds.constant_clustering();
    std::cout << "Reference point cloud clustering." << std::endl;
}

void parallel_encoder::clusters_matching(const std::string& aux_name)
{
    std::string name;
    for (int i = 1; i <= 8; i++)
        name.insert(name.begin(), aux_name[aux_name.size() - i]);
    std::ofstream outname("mse/" + name);
    this->point_clouds.centroid_alignment();
    float mse = point_clouds.total_base_icp();
    outname << mse << std::endl;
    //std::cout << "Total icp registration score : " << mse << std::endl;

    std::vector<Cloud> subclouds;
    this->point_clouds.cluster_matching(subclouds, outname);
    //std::cout << "Clusters matching average mse : " << avgmse << std::endl;

    this->clusters.resize(subclouds.size());
    for (size_t i = 0; i < subclouds.size(); i++)
    {
        this->clusters[i].resize(2);
        this->clusters[i][0].resize(subclouds[i].ref_point_cloud.size());
        this->clusters[i][1].resize(subclouds[i].point_cloud.size());
        for (size_t j = 0; j < subclouds[i].ref_point_cloud.size(); j++)
            this->clusters[i][0][j] = subclouds[i].ref_point_cloud[j];
        for (size_t j = 0; j < subclouds[i].point_cloud.size(); j++)
            this->clusters[i][1][j] = subclouds[i].point_cloud[j];
        this->trans.push_back(subclouds[i].transformation_matrix);
    }
}

void parallel_encoder::clusters_compression(const std::string& aux_name, const std::string& geo_name, const std::string& color_name)
{
    std::ofstream aux_file(aux_name);
    aux_file << this->clusters.size() << std::endl;
    std::ofstream geo_file(geo_name, std::ios::binary);

    for (size_t i = 0; i < this->clusters.size(); i++)
    {
        Octree tree;
        pcl::PointXYZ center = tree.set_input_cloud(clusters[i]);
        int str_cnt = tree.compression(geo_file);
        tree.color_compression(this->colors);
        aux_file << str_cnt << std::endl;
        aux_file << center.x << " " << center.y << " " << center.z << " " << tree.tree_range << std::endl;
        aux_file << this->trans[i] << std::endl;
    }
    //turbo_jpeg_encoder(color_name, this->colors, JPEG_COMPRESSION_QUALITY);
    jpeg_encoder(color_name, this->colors, JPEG_COMPRESSION_QUALITY);
}

void parallel_encoder::encoder(const std::string& reffilename, const std::string& filename, const std::string& aux_name,
                               const std::string& geo_name, const std::string& color_name)
{
    this->set_ref_point_cloud(reffilename);
    this->set_point_cloud(filename);

    this->clusters_generating();
    this->clusters_matching(aux_name);

    this->clusters_compression(aux_name, geo_name, color_name);
}