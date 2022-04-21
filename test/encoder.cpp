//
// Created by 10462 on 2022/3/23.
//

#include "parallel_encoder.h"
#include <thread>
#include <mutex>
#include <queue>
std::queue<int> frame_pool;
std::mutex pool_mutex;
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
void proc()
{
    while (true)
    {
        int i;
        bool flag = false;
        pool_mutex.lock();
        if (!frame_pool.empty())
        {
            i = frame_pool.front();
            frame_pool.pop();
            flag = true;
        }
        pool_mutex.unlock();

        if (!flag)
            return;

        parallel_encoder enc;
        enc.encoder("../data/" + data_name[config] + "/" + data_name[config] + "_vox10_" + convert_str(i) + ".ply",
                    "../data/" + data_name[config] + "/" + data_name[config] + "_vox10_" + convert_str(i + 1) + ".ply",
                    "../data/result/" + data_name[config] + "/auxiliary/aux_" + convert_str(i) + ".dat",
                    "../data/result/" + data_name[config] + "/geometry/geo_" + convert_str(i) + ".dat",
                    "../data/result/" + data_name[config] + "/color/color_" + convert_str(i) + ".jpg");
    }
}

int main()
{
    struct timeval tim1{}, tim2{};
    gettimeofday(&tim1, nullptr);
    for (int i = frame_begin[config]; i < frame_begin[config] + 300; i += 2)
        frame_pool.push(i);

    std::thread threads[50];
    for (auto & i : threads)
        i = std::thread(proc);
    for (auto & th : threads)
        th.join();

    gettimeofday(&tim2, nullptr);
    std::cout << "Task complete, using " << (tim2.tv_sec - tim1.tv_sec) / 60 << "m " << (tim2.tv_sec - tim1.tv_sec) % 60 << "s." << std::endl;
    return 0;
}