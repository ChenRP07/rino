//
// Created by 10462 on 2022/3/9.
//

#include "Cloud.h"

int main()
{
    std::string filename = "../data/loot/";
    Cloud pc;
    pc.set_ref_point_cloud(filename + "loot_vox10_1030.ply");
    pc.set_point_cloud(filename + "loot_vox10_1034.ply");
    std::cout << "read cloud" << std::endl;
    float mse = pc.total_base_icp();
    std::cout << "total mse : " << mse << std::endl;
    pc.constant_clustering();
    std::vector<Cloud> subclouds;
    pc.cluster_matching(subclouds);
    int cnt = 0;
    for (size_t i = 0; i < subclouds.size(); i++)
        cnt += subclouds[i].point_cloud.size();
    std::cout << cnt << " " << pc.point_cloud.size() << std::endl;
    return 0;
}