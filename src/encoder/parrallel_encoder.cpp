//
// Created by 10462 on 2022/3/22.
//

#include "parallel_encoder.h"

void parallel_encoder::set_ref_point_cloud(std::string filename)
{
    this->point_clouds.set_ref_point_cloud(filename);
}

void parallel_encoder::set_point_cloud(std::string filename)
{
    this->point_clouds.set_point_cloud(filename);
}

