#include "Octree.h"
using namespace pcl;
using namespace std;

float bounding_range(PointCloud<PointXYZRGB>& source)
{
    float maxx = FLT_MIN, maxy = FLT_MIN, maxz = FLT_MIN;
    float minx = FLT_MAX, miny = FLT_MAX, minz = FLT_MAX;
    for (int i = 0; i < source.size(); i++)
    {
        if (source[i].x > maxx)
            maxx = source[i].x;
        if (source[i].x < minx)
            minx = source[i].x;
        if (source[i].y > maxy)
            maxy = source[i].y;
        if (source[i].y < miny)
            miny = source[i].y;
        if (source[i].z > maxz)
            maxz = source[i].z;
        if (source[i].z < minz)
            minz = source[i].z;
    }
    return max(max(maxx - minx, maxy - miny), maxz - minz);
}
float cloud_mse(PointCloud<PointXYZRGB>& source, PointCloud<PointXYZRGB>& target)
{
    float target_mse = 0.0f, source_mse = 0.0f;

    /* search method kdtree */
    KdTreeFLANN<PointXYZRGB> source_tree, target_tree;
    source_tree.setInputCloud(source.makeShared());
    target_tree.setInputCloud(target.makeShared());

    /* do nearest neighbor search */
    for (int i = 0; i < source.size(); i++)
    {
        vector<int> _idx_(1);
        vector<float> _dis_(1);
        target_tree.nearestKSearch(source[i], 1, _idx_, _dis_);
        source_mse += _dis_[0];
    }

    for (int i = 0; i < target.size(); i++)
    {
        vector<int> _idx_(1);
        vector<float> _dis_(1);
        source_tree.nearestKSearch(target[i], 1, _idx_, _dis_);
        target_mse += _dis_[0];
    }

    source_mse /= source.size();
    target_mse /= target.size();
    return max(source_mse, target_mse);
}

/* this function compute geometry psnr for two point clouds */
float psnr_geo(PointCloud<PointXYZRGB>& source, PointCloud<PointXYZRGB>& target)
{
    /* get the bounding box range for two clouds */
    float information = max(bounding_range(source), bounding_range(target));

    /* get the maximum mse */
    float mse = cloud_mse(source, target);

    cout << "Information : " << information << endl;
    cout << "MSE : " << mse << endl;
    float psnr = 10.0f * log10(information * information / mse);

    return psnr;
}

int main()
{
    std::ofstream outfile("../test/psnr.txt");
    for (int i = 1000; i < 1000 + 300; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> cloud1;
        pcl::io::loadPLYFile("../test/cloud/loot_vox10_" + std::to_string(i) + ".ply", cloud);
        pcl::io::loadPLYFile("../data/loot/loot_vox10_" + std::to_string(i) + ".ply", cloud1);
        outfile << i << " " << psnr_geo(cloud, cloud1) << std::endl;
    }
    return 0;
}