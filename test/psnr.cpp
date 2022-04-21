#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cfloat>
#include <iostream>
#include <cmath>
#include <vector>
std::string data_name[4] = {"loot", "longdress", "soldier", "redandblack"};
int frame_begin[4] = {1000, 1051, 536, 1450};

std::string convert_str(int i)
{
    std::string str = std::to_string(i);
    if (i < 1000)
        str.insert(str.begin(), '0');
    return str;
}
float bounding_range(pcl::PointCloud<pcl::PointXYZRGB>& source)
{
    float maxx = FLT_MIN, maxy = FLT_MIN, maxz = FLT_MIN;
    float minx = FLT_MAX, miny = FLT_MAX, minz = FLT_MAX;
    for (auto & i : source)
    {
        if (i.x > maxx)
            maxx = i.x;
        if (i.x < minx)
            minx = i.x;
        if (i.y > maxy)
            maxy = i.y;
        if (i.y < miny)
            miny = i.y;
        if (i.z > maxz)
            maxz = i.z;
        if (i.z < minz)
            minz = i.z;
    }
    return std::max(std::max(maxx - minx, maxy - miny), maxz - minz);
}

float color_y(pcl::PointXYZRGB& p)
{
    return 0.2126f * (float)p.r + 0.7152f * (float)p.g + 0.0722f * (float)p.b;
}

float color_cb(pcl::PointXYZRGB& p)
{
    return (-0.2126f * (float)p.r - 0.7152f * (float)p.g + 0.9278f * (float)p.b) / 1.8556f;
}

float color_cr(pcl::PointXYZRGB& p)
{
    return (0.7874f * (float)p.r - 0.7152f * (float)p.g - 0.0722f * (float)p.b) / 1.5748f;
}

void cloud_mse(pcl::PointCloud<pcl::PointXYZRGB>& source, pcl::PointCloud<pcl::PointXYZRGB>& target,
               float& geo_mse, float& y_mse, float& u_mse, float& v_mse)
{
    float target_mse = 0.0f, source_mse = 0.0f;

    pcl::KdTreeFLANN<pcl::PointXYZRGB> source_tree, target_tree;
    source_tree.setInputCloud(source.makeShared());
    target_tree.setInputCloud(target.makeShared());

    float source_y_mse = 0.0f, target_y_mse = 0.0f,
          source_u_mse = 0.0f, target_u_mse = 0.0f,
          source_v_mse = 0.0f, target_v_mse = 0.0f;
    for (auto & i : source)
    {
        std::vector<int> _idx_(1);
        std::vector<float> _dis_(1);
        target_tree.nearestKSearch(i, 1, _idx_, _dis_);
        source_mse += _dis_[0];
        source_y_mse += (float)std::pow(color_y(i) - color_y(target[_idx_[0]]), 2);
        source_u_mse += (float)std::pow(color_cb(i) - color_cb(target[_idx_[0]]), 2);
        source_v_mse += (float)std::pow(color_cr(i) - color_cr(target[_idx_[0]]), 2);
    }

    for (auto & i : target)
    {
        std::vector<int> _idx_(1);
        std::vector<float> _dis_(1);
        source_tree.nearestKSearch(i, 1, _idx_, _dis_);
        target_mse += _dis_[0];
        target_y_mse += (float)std::pow(color_y(i) - color_y(source[_idx_[0]]), 2);
        target_u_mse += (float)std::pow(color_cb(i) - color_cb(source[_idx_[0]]), 2);
        target_v_mse += (float)std::pow(color_cr(i) - color_cr(source[_idx_[0]]), 2);
    }

    source_y_mse /= (float)source.size(), source_u_mse /= (float)source.size(), source_v_mse /= (float)source.size();
    target_y_mse /= (float)target.size(), target_u_mse /= (float)target.size(), target_v_mse /= (float)target.size();
    source_mse /= (float)source.size(), target_mse /= (float)target.size();

    y_mse = std::max(source_y_mse, target_y_mse), u_mse = std::max(source_u_mse, target_u_mse), v_mse = std::max(source_v_mse, target_v_mse);
    geo_mse = std::max(source_mse, target_mse);
}

/* this function compute geometry psnr for two point clouds */
void psnr(pcl::PointCloud<pcl::PointXYZRGB>& source, pcl::PointCloud<pcl::PointXYZRGB>& target,
               float& psnr_geo, float& psnr_y, float& psnr_u, float& psnr_v)
{
    /* get the bounding box range for two clouds */
    float information = std::max(bounding_range(source), bounding_range(target));
    float geo_mse = 0.0f, y_mse = 0.0f, u_mse = 0.0f, v_mse = 0.0f;
    /* get the maximum mse */
    cloud_mse(source, target, geo_mse, y_mse, u_mse, v_mse);
    psnr_geo = 10.0f * std::log10(information * information / geo_mse);
    psnr_y = 10.0f * std::log10(255.0f * 255.0f / y_mse);
    psnr_u = 10.0f * std::log10(255.0f * 255.0f / u_mse);
    psnr_v = 10.0f * std::log10(255.0f * 255.0f / v_mse);
}

#include <queue>
#include <thread>
#include <mutex>
std::queue<int> task;
std::mutex file_mutex, task_mutex;
std::ofstream outfile("psnr.txt");
int config = 0;
void proc()
{
    while (true)
    {
        int frame;
        bool flag = false;
        task_mutex.lock();
        if (!task.empty())
        {
            frame = task.front();
            task.pop();
            flag = true;
        }
        task_mutex.unlock();
        if (!flag)
            return;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> cloud1;
        pcl::io::loadPLYFile("../data/" + data_name[config] + "/" + data_name[config] + "_vox10_" + convert_str(frame) + ".ply", cloud);
        pcl::io::loadPLYFile("../data/result/" + data_name[config] + "/cloud/" + data_name[config] + "_vox10_" + convert_str(frame) + ".ply", cloud1);
        float psnr_geo, psnr_y, psnr_u, psnr_v;
        psnr(cloud, cloud1, psnr_geo, psnr_y, psnr_u, psnr_v);
        file_mutex.lock();
        outfile << frame << " "
                << psnr_geo << " "
                << psnr_y << " "
                << psnr_u << " "
                << psnr_v << " "
                << std::endl;
        file_mutex.unlock();
    }
}
int main()
{
    for (int i = frame_begin[config]; i < frame_begin[config] + 300; i++)
        task.push(i);
    std::thread ths[50];
    for (auto & th : ths)
        th = std::thread(proc);
    for (auto& th : ths)
        th.join();
    return 0;
}