#include "Octree.h"

cloud_range::cloud_range(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds)
{
    max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;
    min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;

    for (auto & cloud : clouds)
    {
        for (auto & j : cloud)
        {
            max_x = max_x > j.x ? max_x : j.x;
            max_y = max_y > j.y ? max_y : j.y;
            max_z = max_z > j.z ? max_z : j.z;

            min_x = min_x < j.x ? min_x : j.x;
            min_y = min_y < j.y ? min_y : j.y;
            min_z = min_z < j.z ? min_z : j.z;
        }
    }
}

float cloud_range::max_range() const
{
    return std::max(std::max(this->max_x - this->min_x, this->max_y - this->min_y),
                    this->max_z - this->min_z);
}

bool clouds_empty(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds)
{
    for (auto & cloud : clouds)
        if (!cloud.empty())
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
    this->tree_range = (float)std::pow(2.0f, log_range);
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
    auto temp = (uint8_t)occ;
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
    /* occupation rate 0x00 */
    uint8_t occupy = 0x00;
    /* if there is no point, return 0 */
    if (clouds_empty(clouds))
    {
        return 0;
    }
    /* leaf node */
    else if (Res <= this->min_resolution)
    {
        /* divide to eight parts */
        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>>>
                subclouds(8,std::vector<pcl::PointCloud<pcl::PointXYZRGB>>(clouds.size()));
        cloud_segmentation(clouds, subclouds, center);

        /* for each point cloud */
        for (size_t i = 0; i < subclouds[0].size(); i++)
        {
            uint8_t occ = 0x00; /* occupation rate */
            std::vector<point_color> colors;
            for (size_t j = 0; j < 7; j++)
            {
                if (subclouds[j][i].size() == 0)
                    occ |= 0;
                else
                {
                    occ |= 1;
                    colors.emplace_back(subclouds[j][i], j);
                }
                occ <<= 1;
            }
            if (subclouds[7][i].size() == 0)
                occ |= 0;
            else
            {
                occ |= 1;
                colors.emplace_back(subclouds[7][i], 7);
            }
            this->leafs[i].push_back(occ);
            if (i == 0)
                this->colors_i.push_back(colors);
            else if (i == 1)
                this->colors_p.push_back(colors);
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
}

pcl::PointXYZ Octree::set_input_cloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds)
{
    /* update the tree_range */
    cloud_range clouds_range(clouds);
    this->tree_range = clouds_range.max_range();
    int log_range = std::ceil(std::log2(this->tree_range));
    this->tree_range = (float)std::pow(2.0f, log_range);

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
    std::string all_data;
    all_data += (char)this->tree.size();

    for (auto & i : this->tree)
        for (unsigned char j : i)
            all_data += (char)j;

    std::vector<uint8_t> merge_data(this->leafs[0].size());
    std::vector<int> ind_idx;
    std::vector<uint8_t> ind_data;
    for (size_t i = 0; i < this->leafs[0].size(); i++)
    {
        if (average_hamming_distance(this->leafs[0][i], this->leafs[1][i]) <= MIN_MERGE_HAMMING_DISTANCE)
        {
            merge_data[i] = (this->leafs[0][i] | this->leafs[1][i]);
            merge_color(this->colors_i[i], this->colors_p[i]);
        }
        else
        {
            merge_data[i] = this->leafs[0][i];
            ind_idx.push_back((int)i);
            ind_data.push_back(this->leafs[1][i]);
        }
    }

    for (uint8_t i : merge_data)
        all_data += (char)i;
    int last_idx = 0;
    for (size_t i = 0; i < ind_idx.size(); i++)
    {
        int mis_idx = ind_idx[i] - last_idx;
        if (mis_idx > 127)
        {
            uint8_t res_h = 0x80;
            uint8_t res_l = 0x00;
            res_h |= (uint8_t)(mis_idx / 256);
            res_l = (uint8_t)(mis_idx % 256);
            all_data.push_back((char)res_h);
            all_data.push_back((char)res_l);
            all_data.push_back((char)ind_data[i]);
        }
        else
        {
            auto res = (uint8_t)mis_idx;
            all_data.push_back((char)res);
            all_data.push_back((char)ind_data[i]);
        }
        last_idx = ind_idx[i];
    }

    std::string all_data_result;
    CompressString(all_data, all_data_result, 3);
    outfile << all_data_result;
    return (int)all_data_result.size();
}

void Octree::decompression(std::string &all_data_result)
{
    /* decompress by zstd */
    std::string all_data;
    DecompressString(all_data_result, all_data);

    /* generate an octree */
    size_t data_idx = 0;
    int tree_height = (int) (uint8_t)all_data[data_idx++];

    this->tree.resize(tree_height);
    this->leafs.resize(2);
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

    for (int i = 0; i < node_cnt; i++)
        leafs[0].push_back((uint8_t)all_data[data_idx]),
        leafs[1].push_back((uint8_t)all_data[data_idx++]);
    std::vector<int> ind_idx;
    std::vector<uint8_t> ind_data;
    int last_idx = 0;
    while (data_idx < all_data.size())
    {
        int idx;
        if ((all_data[data_idx] & 0x80) == 0)
            idx = (uint8_t)all_data[data_idx++];
        else
        {
            int idx_h = (uint8_t)all_data[data_idx++] & 0x7f;
            int idx_l = (uint8_t)all_data[data_idx++];
            idx = idx_h * 256 + idx_l;
        }
        ind_idx.push_back(last_idx + idx);
        ind_data.push_back((uint8_t)all_data[data_idx++]);
        last_idx += idx;
    }

    for (size_t i = 0; i < ind_idx.size(); i++)
        leafs[1][ind_idx[i]] = ind_data[i];
}

void Octree::reconstruct(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &clouds, pcl::PointXYZ center,
                         float Range, std::vector<uint8_t> &colors, int color_index)
{
    clouds.resize(this->leafs.size());

    std::vector<std::vector<pcl::PointXYZ>> tree_centers(this->tree.size() + 1);

    tree_centers[0].push_back(center);
    float Res = Range;
    for (int i = 0; i < (int)tree_centers.size() - 1; i++)
    {
        for (int j = 0; j < (int)tree_centers[i].size(); j++)
        {
            std::vector<int> pos;
            occupation_pos(pos, this->tree[i][j]);
            for (int po : pos)
            {
                pcl::PointXYZ ncenter = new_center(tree_centers[i][j], po, Res / 2);
                tree_centers[i + 1].push_back(ncenter);
            }
        }
        Res /= 2;
    }
    size_t color_idx = color_index;
    for (size_t i = 0; i < this->leafs.size(); i++)
    {
        for (size_t j = 0; j < this->leafs[i].size(); j++)
        {
            std::vector<int> pos;
            occupation_pos(pos, this->leafs[i][j]);

            for (int po : pos)
                clouds[i].push_back(new_point(tree_centers[tree_centers.size() - 1][j], po,
                                              colors[color_idx], colors[color_idx + 1],
                                              colors[color_idx + 2])), color_idx += 3;
        }
    }
}


pcl::PointXYZRGB new_point(pcl::PointXYZ &center, int pos, uint8_t red, uint8_t green, uint8_t blue)
{
    pcl::PointXYZRGB npoint;
    npoint.x = center.x;
    npoint.y = center.y;
    npoint.z = center.z;
    npoint.r = red;
    npoint.g = green;
    npoint.b = blue;
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
    return npoint;
}

float average_hamming_distance(uint8_t occ_i, uint8_t occ_p)
{
    return (float)occupation_cnt((char)(occ_i ^ occ_p)) /
    (float)(std::min(occupation_cnt((char)occ_i),
                     occupation_cnt((char)occ_p)) + 1);
}

point_color::point_color(pcl::PointXYZRGB point, int pos)
{
    this->red = point.r;
    this->green = point.g;
    this->blue = point.b;
    this->node_pos = pos;
}

point_color::point_color(const pcl::PointCloud<pcl::PointXYZRGB>& points, int pos)
{
    this->red = 0, this->green = 0, this->blue = 0;
    for (auto & point : points)
        this->red += point.r, this->green += point.g, this->blue += point.b;
    this->node_pos = pos;
}

point_color::point_color(const point_color &point)
{
    this->red = point.red;
    this->green = point.green;
    this->blue = point.blue;
    this->node_pos = point.node_pos;
}

point_color& point_color::operator=(const point_color &point)
{
    this->red = point.red;
    this->green = point.green;
    this->blue = point.blue;
    this->node_pos = point.node_pos;
    return *this;
}

bool point_color::operator<(const point_color &point) const
{
    return this->node_pos < point.node_pos;
}

void merge_color(std::vector<point_color> &color_i, std::vector<point_color> &color_p)
{

    std::set<int> color_i_cnt, color_p_cnt;
    for (auto & i : color_i)
        color_i_cnt.insert(i.node_pos);
    for (auto & p : color_p)
        color_p_cnt.insert(p.node_pos);
    for (auto & i : color_i)
        if (!color_p_cnt.count(i.node_pos))
            color_p.push_back(i);
    for (auto & p : color_p)
        if (!color_i_cnt.count(p.node_pos))
            color_i.push_back(p);
    std::sort(color_i.begin(), color_i.end());
    std::sort(color_p.begin(), color_p.end());
}

void Octree::color_compression(std::vector<uint8_t> &all_colors)
{
    for (auto & i : this->colors_i)
        for (auto & j : i)
            all_colors.push_back(j.red),
            all_colors.push_back(j.green),
            all_colors.push_back(j.blue);
    for (auto & i : this->colors_p)
        for (auto & j : i)
            all_colors.push_back(j.red),
            all_colors.push_back(j.green),
            all_colors.push_back(j.blue);
}