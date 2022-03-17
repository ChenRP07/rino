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
    this->tree_range = 1024.0f;
}

Octree::Octree(float range)
{
    int log_range = std::ceil(std::log2(range));
    this->tree_range = std::pow(2.0f, log_range);
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

int occupation_cnt(char occ)
{
    int cnt = 0;
    uint8_t temp = (uint8_t)occ;
    while (temp != 0)
    {
        cnt += temp % 2;
        temp /= 2;
    }
    return cnt;
}

uint8_t occupation_table[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

void occupation_pos(std::vector<int> &pos, uint8_t occ)
{
    for (int i = 0; i < 8; i++)
        if ((occ & occupation_table[i]) != 0)
            pos.push_back(i);
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
        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>>>
                subclouds(8,std::vector<pcl::PointCloud<pcl::PointXYZRGB>>(clouds.size()));
        cloud_segmentation(clouds, subclouds, center);
        
        for (size_t i = 0; i < subclouds[0].size(); i++)
        {
            uint8_t occ = 0x00;
            for (size_t j = 0; j < 7; j++)
            {
                if (subclouds[j][i].size() == 0)
                    occ |= 0;
                else
                    occ |= 1;
                occ <<= 1;
            }
            if (subclouds[7][i].size() == 0)
                occ |= 0;
            else
                occ |= 1;
            this->leafs[i].push_back(occ);
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

pcl::PointXYZ Octree::set_input_cloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds)
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
    center.x = std::floor((clouds_range.max_x + clouds_range.min_x) / 2);
    center.y = std::floor((clouds_range.max_y + clouds_range.min_y) / 2);
    center.z = std::floor((clouds_range.max_z + clouds_range.min_z) / 2);

    this->tree.resize(cnt - 1);
    this->leafs.resize(clouds.size());
    this->add_tree_node(clouds, 1, this->tree_range, center);

    return center;
}

int Octree::compression(std::ofstream &outfile)
{
    //int leaf_bit = std::ceil(std::log2(std::pow(this->min_resolution, 3)));
    //int residual_bit = std::ceil(std::log2(2 * std::ceil(this->min_resolution / 2)));

    std::string all_data;
    all_data += (char)this->tree.size();

    for (size_t i = 0; i < this->tree.size(); i++)
        for (size_t j = 0; j < this->tree[i].size(); j++)
            all_data += (char)this->tree[i][j];

    for (size_t i = 0; i < this->leafs.size(); i++)
        for (size_t j = 0; j < this->leafs[i].size(); j++)
        {
//            int temp = this->leafs[i][j];
//            while (temp >= 255)
//            {
//                all_data += (char)0xff;
//                temp -= 254;
//            }
//            all_data += (char)temp;
            all_data += this->leafs[i][j];
        }

    std::string all_data_result = "";
    CompressString(all_data, all_data_result, 3);
    outfile << all_data_result;
    return all_data_result.size();
}

void Octree::decompression(std::ifstream &infile, int total_size, int GOF)
{
    /* read from file */
    std::string all_data_result;
    // std::ostringstream buf;
    // buf << infile.rdbuf();
    // all_data_result = buf.str();
    char temp;
    while(total_size--)
    {
        infile.get(temp);
        all_data_result += temp;
    }
    /* decompress by zstd */
    std::string all_data;
    DecompressString(all_data_result, all_data);

    /* generate an octree */
    size_t data_idx = 0;
    int tree_height = (int) all_data[data_idx++];

    this->tree.resize(tree_height);
    this->leafs.resize(GOF);
    //this->points.resize(GOF);

    int node_cnt = 1;
    for (int i = 0; i < tree_height; i++)
    {
        int new_cnt = 0;
        for (int j = 0; j < node_cnt; j++)
        {
            new_cnt += occupation_cnt(all_data[data_idx]);
            this->tree[i].push_back((uint8_t)all_data[data_idx++]);
        }
        node_cnt = new_cnt;
    }

    for (int i = 0; i < GOF; i++)
    {
        for (int j = 0; j < node_cnt; j++)
        {
            // int size_temp = 0;
            // while (all_data[data_idx] == (char)0xff)
            // {
            //     size_temp += 254;
            //     data_idx++;
            // }
            // size_temp += (int)(unsigned char)all_data[data_idx++];
            leafs[i].push_back((uint8_t)all_data[data_idx++]);
        }
    }

//    for (int i = 0; i < leafs.size(); i++)
//    {
//        for (int j = 0; j < leafs[i].size(); j++)
//        {
//            for (int k = 0; k < leafs[i][j] * 3; k++)
//                points[i].push_back((int)all_data[data_idx++]);
//        }
//    }
}

void Octree::reconstruct(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds, pcl::PointXYZ center,
                         float Range, float min_res)
{
    clouds.resize(this->leafs.size());

    std::vector<std::vector<pcl::PointXYZ>> tree_centers(this->tree.size() + 1);

    tree_centers[0].push_back(center);
    float Res = Range;
    for (int i = 0; i < tree_centers.size() - 1; i++)
    {
        for (int j = 0; j < tree_centers[i].size(); j++)
        {
            std::vector<int> pos;
            occupation_pos(pos, this->tree[i][j]);
            for (int h = 0; h < pos.size(); h++)
            {
                pcl::PointXYZ ncenter = new_center(tree_centers[i][j], pos[h], Res / 2);
                tree_centers[i + 1].push_back(ncenter);
            }
        }
        Res /= 2;
    }


//    for (int i = 0; i < clouds.size(); i++)
//    {
//        int point_residual_idx = 0;
//        for (int j = 0; j < this->leafs[i].size(); j++)
//        {
//            for (int h = 0; h < this->leafs[i][j]; h++)
//            {
//                pcl::PointXYZRGB pts;
//                pts.x = tree_centers[tree_centers.size() - 1][j].x + this->points[i][point_residual_idx++];
//                pts.y = tree_centers[tree_centers.size() - 1][j].y + this->points[i][point_residual_idx++];
//                pts.z = tree_centers[tree_centers.size() - 1][j].z + this->points[i][point_residual_idx++];
//                clouds[i].push_back(pts);
//            }
//        }
//    }

    for (size_t i = 0; i < this->leafs.size(); i++)
    {
        for (size_t j = 0; j < this->leafs[i].size(); j++)
        {
            std::vector<int> pos;
            occupation_pos(pos, this->leafs[i][j]);

            for (size_t k = 0; k < pos.size(); k++)
                clouds[i].push_back(new_point(tree_centers[tree_centers.size() - 1][j], pos[k]));
        }
    }
}

void cloud_merge(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<pcl::PointXYZRGB> &part)
{
    for (size_t i = 0; i < part.size(); i++)
        cloud.push_back(part[i]);
}

int Octree_compression(std::ofstream &outfile, std::vector<Octree> &octrees,
                       std::vector<int> &tree_size)
{
    std::string all_data;
    tree_size.resize(octrees.size());
    for (size_t i = 0; i < octrees.size(); i++)
    {
        int cnt = all_data.size();
        for (size_t j = 0; j < octrees[i].tree.size(); j++)
            for (size_t k = 0; k < octrees[i].tree[j].size(); k++)
                all_data += (char) octrees[i].tree[j][k];
        tree_size[i] = all_data.size() - cnt;
    }

    std::string all_data_result;
    CompressString(all_data, all_data_result, 3);
    outfile << all_data_result;
    return all_data_result.size();
}

int residual_compression(std::ofstream &outfile, std::vector<Octree> &octrees,
                                std::vector<int> &residual_size)
{
    std::string all_data;
    residual_size.resize(octrees.size());
    for (size_t i = 0; i < octrees.size(); i++)
    {
        int cnt = all_data.size();
        for (size_t j = 0; j < octrees[i].leafs.size(); j++)
        {
            for (size_t k = 0; k < octrees[i].leafs[j].size(); k++)
            {
//                int temp = octrees[i].leafs[j][k];
//                while (temp >= 255)
//                {
//                    all_data += (char)0xff;
//                    temp -= 254;
//                }
//                all_data += (char)temp;
                all_data += (char)octrees[i].leafs[j][k];
            }
        }

//        for (size_t j = 0; j < octrees[i].points.size(); j++)
//            for (size_t k = 0; k < octrees[i].points[j].size(); k++)
//                all_data += (char)octrees[i].points[j][k];
        residual_size[i] = all_data.size() - cnt;
    }

    std::string all_data_result;
    CompressString(all_data, all_data_result, 3);
    outfile << all_data_result;
    return all_data_result.size();
}

pcl::PointXYZRGB new_point(pcl::PointXYZ &center, int pos)
{
    pcl::PointXYZRGB npoint;
    npoint.x = center.x;
    npoint.y = center.y;
    npoint.z = center.z;
    if (pos == 0)
    {
        npoint.x += 1.0f;
        npoint.y += 1.0f;
        npoint.z += 1.0f;
    }
    else if (pos == 1)
    {
        npoint.x += 1.0f;
        npoint.y += 1.0f;
    }
    else if (pos == 2)
    {
        npoint.x += 1.0f;
        npoint.z += 1.0f;
    }
    else if (pos == 3)
    {
        npoint.x += 1.0f;
    }
    else if (pos == 4)
    {
        npoint.y += 1.0f;
        npoint.z += 1.0f;
    }
    else if (pos == 5)
    {
        npoint.y += 1.0f;
    }
    else if (pos == 6)
    {
        npoint.z += 1.0f;
    }
    else if (pos == 7);
    return npoint;
}