#ifndef RINO_OCTREE_H
#define RINO_OCTREE_H

#include "Cloud.h"

class Octree
{
public:
    std::vector<std::vector<uint8_t>> tree;
    std::vector<std::vector<int>> points;
    std::vector<std::vector<int>> leafs;

    float min_resolution;
    float tree_range;

};

/*
 * 根据point和leaf的取值范围将其转化为若干位的变量，写成bit流再进行压缩。
 * */
#endif