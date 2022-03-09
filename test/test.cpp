//
// Created by 10462 on 2022/3/9.
//

#include "Cloud.h"

int main()
{
    std::string filename = "../data/loot/";
    Cloud pc;
    pc.set_ref_point_cloud(filename + "loot_vox10_1030.ply");
    std::cout << "read cloud" << std::endl;
    pc.constant_clustering();
    std::cout << "cluster" << std::endl;
    pc.ref_point_cloud_cluster_output(filename + "test/");
    return 0;
}