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
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <csetjmp>
#include <vector>
#include "jpeg/jpeglib.h"

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

extern float NORMAL_ESTIMATION_RADIUS;
extern float FPFH_ESTIMATION_RADIUS;

extern float MIN_MERGE_HAMMING_DISTANCE;

extern int PARALLEL_DECODER_THREAD;

extern int CompressString(const std::string& src, std::string& dst, int compressionlevel);
extern int DecompressString(const std::string& src, std::string& dst);

extern void jpeg_encoder(std::string filename, std::vector<uint8_t>& colors, int quality);
extern void jpeg_decoder(std::string filename, std::vector<uint8_t>& colors);
#endif