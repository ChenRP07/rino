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
float color_y(PointXYZRGB& p)
{
    return 0.2126 * p.r + 0.7152 * p.g + 0.0722 * p.b;
}

float color_cb(PointXYZRGB& p)
{
    return (-0.2126 * p.r - 0.7152 * p.g + 0.9278 * p.b) / 1.8556;
}

float color_cr(PointXYZRGB& p)
{
    return (0.7874 * p.r - 0.7152 * p.g - 0.0722 * p.b) / 1.5748;
}

float color_mse(PointCloud<PointXYZRGB>& source, PointCloud<PointXYZRGB>& target, int type)
{
    float source_mse = 0.0f, target_mse = 0.0f;

    KdTreeFLANN<PointXYZRGB> source_tree, target_tree;
    source_tree.setInputCloud(source.makeShared());
    target_tree.setInputCloud(target.makeShared());

    for (int i = 0; i < source.size(); i++)
    {
        vector<int> _idx_(1);
        vector<float> _dis_(1);

        target_tree.nearestKSearch(source[i], 1, _idx_, _dis_);

        if (type == 0)
            source_mse += pow(color_y(source[i]) - color_y(target[_idx_[0]]), 2);
        else if (type == 1)
            source_mse += pow(color_cb(source[i]) - color_cb(target[_idx_[0]]), 2);
        else if (type == 2)
            source_mse += pow(color_cr(source[i]) - color_cr(target[_idx_[0]]), 2);
    }

    for (int i = 0; i < target.size(); i++)
    {
        vector<int> _idx_(1);
        vector<float> _dis_(1);

        source_tree.nearestKSearch(target[i], 1, _idx_, _dis_);

        if (type == 0)
            target_mse += pow(color_y(target[i]) - color_y(source[_idx_[0]]), 2);
        else if (type == 1)
            target_mse += pow(color_cb(target[i]) - color_cb(source[_idx_[0]]), 2);
        else if (type == 2)
            target_mse += pow(color_cr(target[i]) - color_cr(source[_idx_[0]]), 2);
    }

    source_mse /= source.size();
    target_mse /= target.size();

    return max(source_mse, target_mse);
}

float psnr_y(PointCloud<PointXYZRGB>& source, PointCloud<PointXYZRGB>& target)
{
    float information = 255.0f;

    float mse = color_mse(source, target, 0);

    cout << "Information : " << information << endl;
    cout << "MSE : " << mse << endl;

    float psnr = 10.0f * log10(information * information / mse);

    return psnr;
}

float psnr_cb(PointCloud<PointXYZRGB>& source, PointCloud<PointXYZRGB>& target)
{
    float information = 255.0f;

    float mse = color_mse(source, target, 1);

    cout << "Information : " << information << endl;
    cout << "MSE : " << mse << endl;

    float psnr = 10.0f * log10(information * information / mse);

    return psnr;
}

float psnr_cr(PointCloud<PointXYZRGB>& source, PointCloud<PointXYZRGB>& target)
{
    float information = 255.0f;

    float mse = color_mse(source, target, 2);

    cout << "Information : " << information << endl;
    cout << "MSE : " << mse << endl;

    float psnr = 10.0f * log10(information * information / mse);

    return psnr;
}
int main()
{
//    std::ofstream outfile("../test/psnr.txt");
//    for (int i = 1000; i < 1000 + 300; i++)
//    {
//        pcl::PointCloud<pcl::PointXYZRGB> cloud;
//        pcl::PointCloud<pcl::PointXYZRGB> cloud1;
//        pcl::io::loadPLYFile("../test/cloud/loot_vox10_" + std::to_string(i) + ".ply", cloud);
//        pcl::io::loadPLYFile("../data/loot/loot_vox10_" + std::to_string(i) + ".ply", cloud1);
//        outfile << i << " " << psnr_geo(cloud, cloud1) << std::endl;
//    }
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> cloud1;
        pcl::io::loadPLYFile("../data/loot/loot_vox10_1000.ply", cloud);
        pcl::io::loadPLYFile("../test/cloud/loot_vox10_1000.ply", cloud1);
        std::cout << psnr_geo(cloud, cloud1) << std::endl;
        std::cout << psnr_y(cloud, cloud1) << std::endl;
    return 0;
}