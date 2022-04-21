//
// Created by 10462 on 2022/4/4.
//

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

int main()
{
    long long cnt = 0;
    for (int i = 1450; i < 1450 + 300; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::io::loadPLYFile("../data/redandblack/redandblack_vox10_" + std::to_string(i) + ".ply", cloud);
        cnt += cloud.size();
    }
    std::cout << cnt / 300 << std::endl;
}