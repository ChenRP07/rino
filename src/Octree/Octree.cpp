#include "Octree.h"

cloud_range::cloud_range(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds)
{
    max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;
    min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;

    for (size_t i = 0; i < clouds.size(); i++)
    {
        for (size_t j = 0; j < clouds[i].size(); j++)
        {
            max_x = max_x > clouds[i][j].x ? max_x : clouds[i][j].x;
            max_y = max_y > clouds[i][j].y ? max_y : clouds[i][j].y;
            max_z = max_z > clouds[i][j].z ? max_z : clouds[i][j].z;

            min_x = min_x < clouds[i][j].x ? min_x : clouds[i][j].x;
            min_y = min_y < clouds[i][j].y ? min_y : clouds[i][j].y;
            min_z = min_z < clouds[i][j].z ? min_z : clouds[i][j].z;
        }
    }
}

float cloud_range::max_range()
{
    return std::max(std::max(this->max_x - this->min_x, this->max_y - this->min_y),
                    this->max_z - this->min_z);
}

bool clouds_empty(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds)
{
    for (size_t i = 0; i < clouds.size(); i++)
        if (!clouds[i].empty())
            return false;
    return true;
}

pcl::PointXYZ new_center(pcl::PointXYZ &center, int pos, float res)
{
    /* new center */
    pcl::PointXYZ ncenter;

    /* for a space cube
       eight parts are
       2 0 \behind 6 4
       3 1         7 5
    */
    if (pos == 0)
    {
        ncenter.x = center.x + res / 2;
        ncenter.y = center.y + res / 2;
        ncenter.z = center.z + res / 2;
    }
    else if (pos == 1)
    {
        ncenter.x = center.x + res / 2;
        ncenter.y = center.y + res / 2;
        ncenter.z = center.z - res / 2;
    }
    else if (pos == 2)
    {
        ncenter.x = center.x + res / 2;
        ncenter.y = center.y - res / 2;
        ncenter.z = center.z + res / 2;
    }
    else if (pos == 3)
    {
        ncenter.x = center.x + res / 2;
        ncenter.y = center.y - res / 2;
        ncenter.z = center.z - res / 2;
    }
    else if (pos == 4)
    {
        ncenter.x = center.x - res / 2;
        ncenter.y = center.y + res / 2;
        ncenter.z = center.z + res / 2;
    }
    else if (pos == 5)
    {
        ncenter.x = center.x - res / 2;
        ncenter.y = center.y + res / 2;
        ncenter.z = center.z - res / 2;
    }
    else if (pos == 6)
    {
        ncenter.x = center.x - res / 2;
        ncenter.y = center.y - res / 2;
        ncenter.z = center.z + res / 2;
    }
    else if (pos == 7)
    {
        ncenter.x = center.x - res / 2;
        ncenter.y = center.y - res / 2;
        ncenter.z = center.z - res / 2;
    }

    return ncenter;
}

Octree::Octree()
{
    this->min_resolution = 4.0f;
    this->tree_range = 1024.0f;
}

Octree::Octree(float range)
{
    this->min_resolution = 4.0f;
    int log_range = std::ceil(std::log2(range));
    this->tree_range = std::pow(2.0f, log_range);
}

Octree::Octree(float res, float range)
{
    this->min_resolution = res;
    this->tree_range = range;
}

void cloud_segmentation(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &cloud,
                                std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>>> &subclouds,
                                pcl::PointXYZ center)
{
    for (size_t i = 0; i < cloud.size(); i++)
    {
        for (size_t j = 0; j < cloud[i].size(); j++)
        {
            int pos = 0;
            pos |= cloud[i][j].x > center.x ? 0 : 1;
            pos <<= 1;
            pos |= cloud[i][j].y > center.y ? 0 : 1;
            pos <<= 1;
            pos |= cloud[i][j].z > center.z ? 0 : 1;
            subclouds[pos][i].push_back(cloud[i][j]);
        }
    }
    cloud.clear();
}

int Octree::add_tree_node(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds, int height, float Res,
                          pcl::PointXYZ center)
{
    uint8_t occupy = 0x00;
    if (clouds_empty(clouds))
    {
        return 0;
    }
    else if (Res <= this->min_resolution)
    {
        for (size_t i = 0; i < clouds.size(); i++)
        {
            this->leafs[i].push_back((unsigned char)clouds[i].size());
            for (size_t j = 0; j < clouds[i].size(); j++)
            {
                this->points[i].push_back(std::ceil(clouds[i][j].x - center.x));
                this->points[i].push_back(std::ceil(clouds[i][j].y - center.y));
                this->points[i].push_back(std::ceil(clouds[i][j].z - center.z));
            }

        }
        return 1;
    }
    else
    {
        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>>>
                subclouds(8,std::vector<pcl::PointCloud<pcl::PointXYZRGB>>(clouds.size()));
        cloud_segmentation(clouds, subclouds, center);

        for (int i = 0; i < 7; i++)
        {
            pcl::PointXYZ ncenter = new_center(center, i, Res / 2);
            occupy |= this->add_tree_node(subclouds[i], height + 1, Res / 2, ncenter);
            occupy <<= 1;
        }
        pcl::PointXYZ ncenter = new_center(center, 7, Res / 2);
        occupy |= this->add_tree_node(subclouds[7], height + 1, Res / 2, ncenter);

        this->tree[height - 1].push_back(occupy);
        return 1;
    }
    return 0;
}

void Octree::set_input_cloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds)
{
    /* update the tree_range */
    cloud_range clouds_range(clouds);
    this->tree_range = clouds_range.max_range();
    int log_range = std::ceil(std::log2(this->tree_range));
    this->tree_range = std::pow(2.0f, log_range);

    /* calculate the tree height */
    int cnt = 1;
    float res = this->tree_range;
    while (res > this->min_resolution)
    {
        cnt++;
        res /= 2.0f;
    }

    pcl::PointXYZ center;
    center.x = (clouds_range.max_x + clouds_range.min_x) / 2;
    center.y = (clouds_range.max_y + clouds_range.min_y) / 2;
    center.z = (clouds_range.max_z + clouds_range.min_z) / 2;

    this->tree.resize(cnt - 1);
    this->add_tree_node(clouds, 1, this->tree_range, center);
}

void Octree::compression(std::ofstream &outfile)
{
    //int leaf_bit = std::ceil(std::log2(std::pow(this->min_resolution, 3)));
    //int residual_bit = std::ceil(std::log2(2 * std::ceil(this->min_resolution / 2)));

    std::string all_data;

    for (size_t i = 0; i < this->tree.size(); i++)
        for (size_t j = 0; j < this->tree[i].size(); j++)
            all_data += (char)this->tree[i][j];

    for (size_t i = 0; i < this->leafs.size(); i++)
        for (size_t j = 0; j < this->leafs[i].size(); j++)
            all_data += (char)this->leafs[i][j];

    for (size_t i = 0; i < this->points.size(); i++)
        for (size_t j = 0; j < this->points[i].size(); j++)
            all_data += (char)this->points[i][j];

    std::string all_data_result = "";
    CompressString(all_data, all_data_result, 3);
    outfile << all_data_result;
}