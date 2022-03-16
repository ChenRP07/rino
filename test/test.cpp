//
// Created by 10462 on 2022/3/16.
//
#include "Octree.h"
int main()
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> cloud(2);
    pcl::io::loadPLYFile("../test/1.ply", cloud[0]);
    pcl::io::loadPLYFile("../test/1b.ply", cloud[1]);
    Octree tree;
    tree.set_input_cloud(cloud);
    std::cout<<tree.min_resolution <<" " <<tree.tree_range<<std::endl;
    std::cout<<tree.leafs[0].size() <<" "<< tree.leafs[1].size()<<std::endl;
    std::ofstream  outfile("../test/data.dat");
    tree.compression(outfile);
}
