//
// Created by 10462 on 2022/3/22.
//

#ifndef RINO_PARALLEL_ENCODER_H
#define RINO_PARALLEL_ENCODER_H

#include "Octree.h"
#include <sys/types.h>
#include <ctime>
#include <utility>

class parallel_encoder
{
public:
    Cloud point_clouds;
    struct timeval time1, time2;

    void set_ref_point_cloud(std::string filename);
    void set_point_cloud(std::string filename);
    void clusters_generating();

};
#endif //RINO_PARALLEL_ENCODER_H
