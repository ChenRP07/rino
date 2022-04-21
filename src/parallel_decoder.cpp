//
// Created by 10462 on 2022/3/21.
//

#include "parallel_decoder.h"

merge_task::merge_task(int pm1, int pm2, int pm3)
{
    this->index = pm1, this->start_index = pm2, this->length = pm3;
}

merge_task::merge_task()
{
    this->index = 0, this->start_index = 0, this->length = 0;
}

merge_task::merge_task(const merge_task &task)
{
    this->index = task.index, this->start_index = task.start_index, this->length = task.length;
}

merge_task & merge_task::operator = (const merge_task &task)
{
    this->index = task.index, this->start_index = task.start_index, this->length = task.length;
    return *this;
}

void parallel_decoder::resize_containers()
{
    this->str_cnt.resize(this->clusters_cnt);
    this->tree_range.resize(this->clusters_cnt);
    this->centers.resize(this->clusters_cnt);
    this->trans.resize(this->clusters_cnt);
    this->geometry_data_pool.resize(this->clusters_cnt);
    this->all_colors.resize(this->clusters_cnt);
    this->trees.resize(this->clusters_cnt);
    this->point_clouds.resize(2);
    this->point_clusters.resize(clusters_cnt, std::vector<pcl::PointCloud<pcl::PointXYZRGB>>(2));
}
void parallel_decoder::set_aux_information(const std::string& aux_file)
{
    gettimeofday(&this->time1, nullptr);    /* timer1 */
    std::ifstream aux_in(aux_file); /* read clusters count */
    aux_in >> this->clusters_cnt;

    this->resize_containers();  /* allocate memory */

    for (int i = 0; i < this->clusters_cnt; i++) /* for all clusters */
    {
        aux_in >> str_cnt[i];   /* compressed string length */
        aux_in >> centers[i].x >> centers[i].y >> centers[i].z; /* octree center */
        aux_in >> tree_range[i];    /* octree range */
        for (int row = 0; row < 4; row++)   /* transformation matrix */
            for (int col = 0; col < 4; col++)
                aux_in >> trans[i](row, col);
    }
    gettimeofday(&this->time2, nullptr);    /* timer2 and output consumed time */
    std::cout << "Read auxiliary information, using " << std::fixed << std::setprecision(3)
              << (float)(this->time2.tv_sec - this->time1.tv_sec) + (float)(this->time2.tv_usec - this->time1.tv_usec) / 1000000
              << "s." << std::endl;
}

void parallel_decoder::set_geometry_information(const std::string& geo_file)
{
    gettimeofday(&this->time1, nullptr);    /* timer1 */
    std::ifstream geo_in(geo_file);
    for (size_t i = 0; i < str_cnt.size(); i++)
    {
        int total_cnt = str_cnt[i]; /* read counter */
        char temp_char;
        while (total_cnt--) /* read from file */
        {
            geo_in.get(temp_char);
            this->geometry_data_pool[i] += temp_char;
        }
        this->geometry_task_pool.push((int)i);   /* add to queue */
    }
    gettimeofday(&this->time2, nullptr);    /* timer2 and output consumed time */
    std::cout << "Read geometry information, using " << std::fixed << std::setprecision(3)
              << (float)(this->time2.tv_sec - this->time1.tv_sec) + (float)(this->time2.tv_usec - this->time1.tv_usec) / 1000000
              << "s." << std::endl;
}

void parallel_decoder::octree_geometry_construction()
{
    while (true)
    {
        int index;
        bool flag = false;  /* all task complete ? */
        this->geometry_task_pool_mutex.lock();
        if (!this->geometry_task_pool.empty())  /* attempt to get a task */
        {
            index = this->geometry_task_pool.front();   /* success */
            this->geometry_task_pool.pop();
            flag = true;
        }
        this->geometry_task_pool_mutex.unlock();
        if (!flag)  /* no task return */
            break;

        this->trees[index].decompression(this->geometry_data_pool[index]);  /* continue to work */
    }
}

void parallel_decoder::parallel_octree_geometry_construction()
{
    gettimeofday(&this->time1, nullptr);    /* timer1 */

    std::thread threads[PARALLEL_DECODER_THREAD];   /* threads */
    for (int i = 0; i < PARALLEL_DECODER_THREAD; i++)   /* create threads */
        threads[i] = std::thread(&parallel_decoder::octree_geometry_construction, this);
    for (auto & th : threads)   /* wait for destruction */
        th.join();

    gettimeofday(&this->time2, nullptr);    /* timer2 and output consumed time */
    std::cout << "Parallel geometry decompression, using " << std::fixed << std::setprecision(3)
              << (float)(this->time2.tv_sec - this->time1.tv_sec) + (float)(this->time2.tv_usec - this->time1.tv_usec) / 1000000
              << "s, " << PARALLEL_DECODER_THREAD << " threads." << std::endl;
}

void parallel_decoder::set_color_information(std::string color_file)
{
    FILE* fp = fopen(color_file.c_str(), "rb");
    unsigned char* jpeg_image = NULL;
    fseek(fp, 0, SEEK_END);
    unsigned long src_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    jpeg_image = (unsigned char *)malloc(src_size * sizeof(unsigned char));
    fread(jpeg_image, src_size, 1, fp);
    fclose(fp);

    gettimeofday(&this->time1, nullptr);    /* timer1 */
    unsigned int size = turbo_jpeg_decoder(jpeg_image, this->color_info, src_size);
    //jpeg_decoder(std::move(color_file), this->color_info);   /* jpeg decode */
    gettimeofday(&this->time2, nullptr);    /* timer2 and output consumed time */
    std::cout << "JPEG decoder, using " << std::fixed << std::setprecision(3)
              << (float)(this->time2.tv_sec - this->time1.tv_sec) + (float)(this->time2.tv_usec - this->time1.tv_usec) / 1000000
              << "s." << std::endl;
              
    gettimeofday(&this->time1, nullptr);    /* timer1 */

    int color_index = 0;
    for (size_t i = 0; i < this->all_colors.size(); i++)    /* for each cluster */
    {
        int color_cnt = 0;  /* cluster points counter */
        for (auto & leaf : this->trees[i].leafs)    /* for each cloud */
        {
            for (unsigned char h : leaf) /* for each leaf */
                color_cnt += occupation_cnt((char)h);  /* count point number */
        }
        this->all_colors[i] = color_index;  /* allocate memory */
        color_index += (color_cnt * 3);
        this->reconstruction_task_pool.push((int)i);
    }

    gettimeofday(&this->time2, nullptr);    /* timer2 and output consumed time */
    std::cout << "Read color information, using " << std::fixed << std::setprecision(3)
              << (float)(this->time2.tv_sec - this->time1.tv_sec) + (float)(this->time2.tv_usec - this->time1.tv_usec) / 1000000
              << "s." << std::endl;
}

void parallel_decoder::octree_cloud_reconstruction()
{
    while (true)
    {
        int index;
        bool flag = false;  /* all task complete ? */
        this->reconstruction_task_pool_mutex.lock();
        if (!this->reconstruction_task_pool.empty())  /* attempt to get a task */
        {
            index = this->reconstruction_task_pool.front();   /* success */
            this->reconstruction_task_pool.pop();
            flag = true;
        }
        this->reconstruction_task_pool_mutex.unlock();
        if (!flag)  /* no task return */
            break;
        //std::vector<pcl::PointCloud<pcl::PointXYZRGB>> clouds(2);
        this->trees[index].reconstruct(this->point_clusters[index], this->centers[index],
                                       this->tree_range[index], this->color_info,
                                       this->all_colors[index]);
        cloud_transformation(this->point_clusters[index][1], this->trans[index].inverse());
    }
}

void parallel_decoder::parallel_cloud_reconstruction()
{
    gettimeofday(&this->time1, nullptr);    /* timer1 */

    std::thread threads[PARALLEL_DECODER_THREAD];   /* threads */
    for (int i = 0; i < PARALLEL_DECODER_THREAD; i++)   /* create threads */
        threads[i] = std::thread(&parallel_decoder::octree_cloud_reconstruction, this);
    for (auto & th : threads)   /* wait for destruction */
        th.join();

    gettimeofday(&this->time2, nullptr);    /* timer2 and output consumed time */
    std::cout << "Parallel cloud reconstruction, using " << std::fixed << std::setprecision(3)
              << (float)(this->time2.tv_sec - this->time1.tv_sec) + (float)(this->time2.tv_usec - this->time1.tv_usec) / 1000000
              << "s, " << PARALLEL_DECODER_THREAD << " threads." << std::endl;
}

void parallel_decoder::point_cloud_merge()
{
    while (true)
    {
        merge_task itask, ptask;
        bool flag = false;  /* all task complete ? */
        this->cloud_merge_mutex.lock();
        if (!this->icloud_merge_task_pool.empty() && !this->pcloud_merge_task_pool.empty())  /* attempt to get a task */
        {
            itask = this->icloud_merge_task_pool.front();   /* success */
            ptask = this->pcloud_merge_task_pool.front();
            this->icloud_merge_task_pool.pop();
            this->pcloud_merge_task_pool.pop();
            flag = true;
        }
        this->cloud_merge_mutex.unlock();
        if (!flag)  /* no task return */
            break;

        for (size_t i = 0; i < this->point_clusters[itask.index][0].size(); i++)
            this->point_clouds[0][itask.start_index + i] = this->point_clusters[itask.index][0][i];

        for (size_t i = 0; i < this->point_clusters[ptask.index][1].size(); i++)
            this->point_clouds[1][ptask.start_index + i] = this->point_clusters[ptask.index][1][i];
    }
}

void parallel_decoder::parallel_cloud_merge()
{
    gettimeofday(&this->time1, nullptr);    /* timer1 */

    int icloud_cnt = 0;
    for (size_t i = 0; i < this->point_clusters.size(); i++)
    {
        this->icloud_merge_task_pool.push(merge_task((int)i, icloud_cnt, (int)this->point_clusters[i][0].size()));
        icloud_cnt += (int)this->point_clusters[i][0].size();
    }

    int pcloud_cnt = 0;
    for (size_t i = 0; i < this->point_clusters.size(); i++)
    {
        this->pcloud_merge_task_pool.push(merge_task((int)i, pcloud_cnt, (int)this->point_clusters[i][1].size()));
        pcloud_cnt += (int)this->point_clusters[i][1].size();
    }

    this->point_clouds[0].resize(icloud_cnt);
    this->point_clouds[1].resize(pcloud_cnt);

    std::thread threads[PARALLEL_DECODER_THREAD];   /* threads */
    for (int i = 0; i < PARALLEL_DECODER_THREAD; i++)   /* create threads */
        threads[i] = std::thread(&parallel_decoder::point_cloud_merge, this);
    for (auto & th : threads)   /* wait for destruction */
        th.join();

    gettimeofday(&this->time2, nullptr);    /* timer2 and output consumed time */
    std::cout << "Parallel cloud merge, using " << std::fixed << std::setprecision(3)
              << (float)(this->time2.tv_sec - this->time1.tv_sec) + (float)(this->time2.tv_usec - this->time1.tv_usec) / 1000000
              << "s, " << PARALLEL_DECODER_THREAD << " threads." << std::endl;
}

void parallel_decoder::save_point_cloud(const std::string& cloud_name1, const std::string& cloud_name2)
{
    gettimeofday(&this->time1, nullptr);    /* timer1 */
    savePly(cloud_name1, this->point_clouds[0]);
    savePly(cloud_name2, this->point_clouds[1]);
    gettimeofday(&this->time2, nullptr);    /* timer2 and output consumed time */
    std::cout << "Save point cloud, using " << std::fixed << std::setprecision(3)
              << (float)(this->time2.tv_sec - this->time1.tv_sec) + (float)(this->time2.tv_usec - this->time1.tv_usec) / 1000000
              << "s." << std::endl;
}

void parallel_decoder::decoder(const std::string& aux_file, const std::string& geo_file, std::string color_file,
                               const std::string& cloud_name1, const std::string& cloud_name2)
{
    this->set_aux_information(aux_file);

    this->set_geometry_information(geo_file);

    this->parallel_octree_geometry_construction();

    this->set_color_information(std::move(color_file));

    this->parallel_cloud_reconstruction();

    this->parallel_cloud_merge();

    this->save_point_cloud(cloud_name1, cloud_name2);
}

void savePly(const std::string& filename, pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    std::ofstream outfile(filename);
    outfile << "ply" << std::endl;
    outfile << "format ascii 1.0" << std::endl;
    outfile << "comment Version 2, Copyright 2017, 8i Labs, Inc." << std::endl;
    outfile << "comment frame_to_world_scale 0.181985" << std::endl;
    outfile << "comment frame_to_world_translation -31.8478 1.0016 -32.6788" << std::endl;
    outfile << "comment width 1024" << std::endl;
    outfile << "element vertex " << cloud.size() << std::endl;
    outfile << "property float x" << std::endl;
    outfile << "property float y" << std::endl;
    outfile << "property float z" << std::endl;
    outfile << "property uchar red" << std::endl;
    outfile << "property uchar green" << std::endl;
    outfile << "property uchar blue" << std::endl;
    outfile << "end_header" << std::endl;
    for (auto & i : cloud)
        outfile << i.x << " " << i.y << " " << i.z << " "
                << (int)i.r << " " << (int)i.g << " " << (int)i.b << std::endl;
}
