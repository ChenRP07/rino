//
// Created by 10462 on 2022/3/23.
//

#include "parallel_encoder.h"
#include <thread>
#include <mutex>
#include <queue>
std::queue<int> frame_pool;
std::mutex pool_mutex;
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
        enc.encoder("../data/loot/loot_vox10_" + std::to_string(i) + ".ply",
                    "../data/loot/loot_vox10_" + std::to_string(i + 1) + ".ply",
                    "../data/result/loot/auxiliary/aux_" + std::to_string(i) + ".dat",
                    "../data/result/loot/geometry/geo_" + std::to_string(i) + ".dat",
                    "../data/result/loot/color/color_" + std::to_string(i) + ".jpg");
    }
}

int main()
{
    struct timeval tim1{}, tim2{};
    gettimeofday(&tim1, nullptr);
    for (int i = 1000; i < 1000 + 300; i += 2)
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