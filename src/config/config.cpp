#include "config.h"

float MAX_CORRESPONDENCE_DISTANCE = 25.0f;
float TRANSFORMATION_DIFFERENCE_THRESHOLD = 1e-6;
int MAX_ICP_ITERATION = 1000;
float MSE_DIFFERENCE_THRESHOLD = 0.01f;
float MAX_FITNESS_MSE = 100.0f;

float OVERLAP_SQRT_DISTANCE_THRESHOLD = 10.0f;
int OVERLAP_POINTS_THRESHOLD = 10;

float DENSE_CLUSTERING_NEIGHBOR_RADIUS = 5.0f;
int DENSE_CLUSTERING_NEIGHBOR_NUMBER = 4;

int CONSTANT_CLUSTER_NUMBER = 50;

int CompressString(const std::string& src, std::string& dst, int compressionlevel)
{
    size_t const cBuffSize = ZSTD_compressBound(src.size());
    dst.resize(cBuffSize);
    auto dstp = const_cast<void*>(static_cast<const void*>(dst.c_str()));
    auto srcp = static_cast<const void*>(src.c_str());
    size_t const cSize = ZSTD_compress(dstp, cBuffSize, srcp, src.size(), compressionlevel);
    auto code = ZSTD_isError(cSize);
    if (code)
    {
        return code;
    }
    dst.resize(cSize);
    return code;
}

int DecompressString(const std::string& src, std::string& dst)
{
    size_t const cBuffSize = ZSTD_getFrameContentSize(src.c_str(), src.size());

    if (0 == cBuffSize)
    {
        return cBuffSize;
    }

    if (ZSTD_CONTENTSIZE_UNKNOWN == cBuffSize)
    {
        return DecompressString(src, dst);
    }

    if (ZSTD_CONTENTSIZE_ERROR == cBuffSize)
    {
        return -2;
    }

    dst.resize(cBuffSize);
    auto dstp = const_cast<void*>(static_cast<const void*>(dst.c_str()));
    auto srcp = static_cast<const void*>(src.c_str());
    size_t const cSize = ZSTD_decompress(dstp, cBuffSize, srcp, src.size());
    auto code = ZSTD_isError(cSize);
    if (code)
    {
        return code;
    }
    dst.resize(cSize);
    return code;
}
