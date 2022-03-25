//
// Created by 10462 on 2022/3/14.
//
#include "parallel_decoder.h"

int main()
{
    for (int i = 1000; i < 1000 + 300; i += 2)
    {
        std::string aux_file = "../data/result/loot/auxiliary/aux_";
        std::string geo_file = "../data/result/loot/geometry/geo_";
        std::string color_file = "../data/result/loot/color/color_";
        std::string cloud_file = "../data/result/loot/cloud/loot_vox10_";
//        std::string aux_file = "../test/data/aux_";
//        std::string geo_file = "../test/data/outdata_";
//        std::string color_file = "../test/data/outdata_";
//        std::string cloud_file = "../test/cloud/loot_vox10_";
        parallel_decoder cloud_decoder;
        cloud_decoder.decoder(aux_file + std::to_string(i) + ".dat",
                              geo_file + std::to_string(i) + ".dat",
                              color_file + std::to_string(i) + ".jpg",
                              cloud_file + std::to_string(i) + ".ply",
                              cloud_file + std::to_string(i + 1) + ".ply");
    }
}
