/*
 * Copyright (c) Chen Ruopeng, SDU, Univ.
 * All rights reserved.
 *
 * This source code is licensed under the Mozilla Public license (found in the
 * LICENSE file in the root directory of this source tree).
 */
#ifndef RINO_CONFIG_H
#define RINO_CONFIG_H
#include <zstd.h>
#include <string>

extern float MAX_CORRESPONDENCE_DISTANCE;
extern float MSE_DIFFERENCE_THRESHOLD;
extern float TRANSFORMATION_DIFFERENCE_THRESHOLD;
extern int MAX_ICP_ITERATION;
extern float MAX_FITNESS_MSE;

extern float OVERLAP_SQRT_DISTANCE_THRESHOLD;
extern int OVERLAP_POINTS_THRESHOLD;

extern float DENSE_CLUSTERING_NEIGHBOR_RADIUS;
extern int DENSE_CLUSTERING_NEIGHBOR_NUMBER;

extern int CONSTANT_CLUSTER_NUMBER;

extern int CompressString(const std::string& src, std::string& dst, int compressionlevel);
extern int DecompressString(const std::string& src, std::string& dst);
#endif