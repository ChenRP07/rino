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
    pcl::PointXYZ center = tree.set_input_cloud(cloud);
    std::cout<<tree.min_resolution <<" " <<tree.tree_range << " " << tree.tree.size() <<std::endl;
    std::cout<<tree.leafs[0].size() <<" "<< tree.leafs[1].size()<<std::endl;
    std::ofstream outfile("../test/data.dat");
    int cnt = tree.compression(outfile);
    float range = tree.tree_range;
    std::vector<uint8_t> colors;
    tree.color_compression(colors);
    jpeg_encoder("../test/123.jpg", colors, 70);

//    std::vector<uint8_t> all_colors;
//    jpeg_decoder("../test/123.jpg", all_colors);
//    std::ifstream infile("../test/data.dat");
//    Octree tree1;
//    tree1.decompression(infile);
//    std::cout<<tree1.min_resolution <<" " <<tree1.tree_range<<std::endl;
//    std::cout<<tree1.leafs[0].size() <<" "<< tree1.leafs[1].size()<<std::endl;
//    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> clouds(2);
//    tree1.reconstruct(clouds, center, range, all_colors);
//    pcl::io::savePLYFile("../test/2.ply", clouds[0]);
//    pcl::io::savePLYFile("../test/2b.ply", clouds[1]);

}
