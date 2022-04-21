//
// Created by 10462 on 2022/3/14.
//
#include "parallel_decoder.h"
std::string data_name[4] = {"loot", "longdress", "soldier", "redandblack"};
int frame_begin[4] = {1000, 1051, 536, 1450};
int config = 0;
std::string convert_str(int i)
{
    std::string str = std::to_string(i);
    if (i < 1000)
        str.insert(str.begin(), '0');
    return str;
}

int main()
{
    for (int i = frame_begin[config]; i < frame_begin[config] + 300; i += 2)
    {
        parallel_decoder cloud_decoder;
        cloud_decoder.decoder("../data/result/" + data_name[config] + "/auxiliary/aux_" + convert_str(i) + ".dat",
                              "../data/result/" + data_name[config] + "/geometry/geo_" + convert_str(i) + ".dat",
                              "../data/result/" + data_name[config] + "/color/color_" + convert_str(i) + ".jpg",
                              "../data/result/" + data_name[config] + "/cloud/" + data_name[config] + "_vox10_" + convert_str(i) + ".ply",
                              "../data/result/" + data_name[config] + "/cloud/" + data_name[config] + "_vox10_" + convert_str(i + 1) + ".ply");
    }
}
