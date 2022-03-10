#include "Cloud.h"

Cloud::Cloud()
{
    this->transformation_matrix = Eigen::Matrix4f::Identity();
}

int Cloud::set_point_cloud(std::string filename)
{
    if (pcl::io::loadPLYFile(filename, this->point_cloud) == -1)
    {
        std::cerr << "ERROR! Cannot load point cloud from file " << filename << "." << std::endl;
        return -1;
    }

    return 0;
}

int Cloud::set_ref_point_cloud(std::string filename)
{
    if (pcl::io::loadPLYFile(filename, this->ref_point_cloud) == -1)
    {
        std::cerr << "ERROR! Cannot load point cloud from file " << filename << "." << std::endl;
        return -1;
    }

    return 0;
}

void Cloud::change_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> &point_cloud)
{
    this->point_cloud.resize(point_cloud.size());
    for (size_t i = 0; i < point_cloud.size(); i++)
        this->point_cloud[i] = point_cloud[i];
}

void Cloud::change_ref_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> &ref_point_cloud)
{
    this->ref_point_cloud.resize(ref_point_cloud.size());
    for (size_t i = 0; i < ref_point_cloud.size(); i++)
        this->ref_point_cloud[i] = ref_point_cloud[i];
}

void Cloud::centroid_alignment()
{
    /* calculate centroids Cp and Cr for point_cloud and ref_point_cloud */
    pcl::PointXYZ Cp(0.0f, 0.0f, 0.0f), Cr(0.0f, 0.0f, 0.0f);/* centorids */

    for (size_t i = 0; i < this->point_cloud.size(); i++)
    {
        Cp.x += this->point_cloud[i].x;
        Cp.y += this->point_cloud[i].y;
        Cp.z += this->point_cloud[i].z;
    }

    for (size_t i = 0; i < this->ref_point_cloud.size(); i++)
    {
        Cr.x += this->ref_point_cloud[i].x;
        Cr.y += this->ref_point_cloud[i].y;
        Cr.z += this->ref_point_cloud[i].z;
    }

    Cp.x /= this->point_cloud.size(), Cp.y /= this->point_cloud.size(), Cp.z /= this->point_cloud.size();
    Cr.x /= this->ref_point_cloud.size(), Cr.y /= this->ref_point_cloud.size(), Cr.z /= this->ref_point_cloud.size();

    /* for each point, do a translation */
    for (size_t i = 0; i < this->point_cloud.size(); i++)
    {
        this->point_cloud[i].x = this->point_cloud[i].x - Cp.x + Cr.x;
        this->point_cloud[i].y = this->point_cloud[i].y - Cp.y + Cr.y;
        this->point_cloud[i].z = this->point_cloud[i].z - Cp.z + Cr.z;
    }

    /* then record this translation to transformation_matrix */
    this->transformation_matrix(0, 3) = this->transformation_matrix(0, 3) - Cp.x + Cr.x;
    this->transformation_matrix(1, 3) = this->transformation_matrix(1, 3) - Cp.y + Cr.y;
    this->transformation_matrix(2, 3) = this->transformation_matrix(2, 3) - Cp.z + Cr.z;
    this->transformation_matrix(3, 3) = 1.0f;

}

float Cloud::total_base_icp()
{
    /* icp algorithm, transform point_cloud to get max overlap with ref_point_cloud */
    pcl::PointCloud<pcl::PointXYZRGB> result;

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(this->point_cloud.makeShared());
    icp.setInputTarget(this->ref_point_cloud.makeShared());

    icp.setMaxCorrespondenceDistance(MAX_CORRESPONDENCE_DISTANCE);
    icp.setEuclideanFitnessEpsilon(MSE_DIFFERENCE_THRESHOLD);
    icp.setMaximumIterations(MAX_ICP_ITERATION);
    icp.setTransformationEpsilon(TRANSFORMATION_DIFFERENCE_THRESHOLD);

    icp.align(result);
    Eigen::Matrix4f trans = icp.getFinalTransformation();

    this->point_cloud.swap(result);
    this->transformation_matrix = trans * transformation_matrix;
    //std::cout << "ICP Convergence : " << icp.getFitnessScore() << "." << std::endl;
    return icp.getFitnessScore();
}

void Cloud::overlap_segmentation(Cloud &overlap, Cloud &nonoverlap)
{
    /* kdtree to do nearest neighbor search */
    pcl::KdTreeFLANN<pcl::PointXYZRGB> tree, ref_tree;

    tree.setInputCloud(this->point_cloud.makeShared());
    ref_tree.setInputCloud(this->ref_point_cloud.makeShared());

    /* search neighbors in r-radius, add point according to neighbors number */
    for (size_t i = 0; i < this->point_cloud.size(); i++)
    {
        std::vector<int> _idx_;
        std::vector<float> _dis_;
        ref_tree.radiusSearch(this->point_cloud[i], OVERLAP_SQRT_DISTANCE_THRESHOLD, _idx_, _dis_);
        if (_idx_.size() > OVERLAP_POINTS_THRESHOLD)
            overlap.point_cloud.push_back(this->point_cloud[i]);
        else
            nonoverlap.point_cloud.push_back(this->point_cloud[i]);
    }

    for (size_t i = 0; i < this->ref_point_cloud.size(); i++)
    {
        std::vector<int> _idx_;
        std::vector<float> _dis_;
        tree.radiusSearch(this->ref_point_cloud[i], OVERLAP_SQRT_DISTANCE_THRESHOLD, _idx_, _dis_);
        if (_idx_.size() > OVERLAP_POINTS_THRESHOLD)
            overlap.ref_point_cloud.push_back(this->point_cloud[i]);
        else
            nonoverlap.ref_point_cloud.push_back(this->point_cloud[i]);
    }

    overlap.transformation_matrix = this->transformation_matrix;
    nonoverlap.transformation_matrix = this->transformation_matrix;
}

void Cloud::dense_clustering()
{
    pcl::KdTreeFLANN<pcl::PointXYZRGB> ref_tree;
    ref_tree.setInputCloud(this->ref_point_cloud.makeShared());

    this->ref_cluster_index.resize(this->ref_point_cloud.size(), -1);

    int ref_cluster_id = 1;
    for (size_t i = 0; i < this->ref_point_cloud.size(); i++)
        if (ref_cluster_index[i] == -1)
            if (dense_cluster_expand(this->ref_point_cloud, ref_tree, ref_cluster_index, i, ref_cluster_id))
                ref_cluster_id++;
}

void Cloud::constant_clustering()
{
    pcl::PointCloud<pcl::PointXYZRGB> centroids;
    constant_cluster_centroid(this->ref_point_cloud, centroids);
    this->ref_cluster_index.resize(this->ref_point_cloud.size(), -1);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> tree;
    tree.setInputCloud(centroids.makeShared());
    for (size_t i = 0; i < this->ref_point_cloud.size(); i++)
    {
        std::vector<int> _idx_(1);
        std::vector<float> _dis_(1);
        tree.nearestKSearch(this->ref_point_cloud[i], 1, _idx_, _dis_);
        this->ref_cluster_index[i] = _idx_[0];
    }
}

void Cloud::ref_point_cloud_cluster_output(std::string address)
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> ref_clusters(
            *std::max_element(this->ref_cluster_index.begin(),
                              this->ref_cluster_index.end()) + 1);

    for (size_t i = 0; i < ref_point_cloud.size(); i++)
        ref_clusters[ref_cluster_index[i]].push_back(ref_point_cloud[i]);

    for (size_t i = 0; i < ref_clusters.size(); i++)
    {
        std::uint8_t r = rand() % 255;
        std::uint8_t g = rand() % 255;
        std::uint8_t b = rand() % 255;
        for (size_t j = 0; j < ref_clusters[i].size(); j++)
        {
            ref_clusters[i][j].r = r;
            ref_clusters[i][j].g = g;
            ref_clusters[i][j].b = b;
        }
    }

    for (size_t i = 0; i < ref_clusters.size(); i++)
        pcl::io::savePLYFile(address + std::to_string(i) + ".ply", ref_clusters[i]);

}


void Cloud::cluster_matching(std::vector<Cloud> &subclouds)
{
    /* divide ref_point_cloud to several clusters */
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> ref_clusters(
            *std::max_element(this->ref_cluster_index.begin(),
                              this->ref_cluster_index.end()) + 1);

    for (size_t i = 0; i < ref_point_cloud.size(); i++)
        ref_clusters[ref_cluster_index[i]].push_back(ref_point_cloud[i]);
    for (auto i = ref_clusters.begin(); i != ref_clusters.end();)
        if (i->size() == 0)
            ref_clusters.erase(i);
        else
            i++;
    subclouds.clear();
    subclouds.resize(ref_clusters.size());

    /* use these clusters to do icp */
    for (size_t i = 0; i < ref_clusters.size(); i++)
    {
        //std::cout<<ref_clusters[i].size()<<std::endl;
        local_icp(ref_clusters[i], this->point_cloud, subclouds[i]);
        //std::cout << "cluster " << i << " : " << mse << std::endl;
        //subclouds[i].transformation_matrix = subclouds[i].transformation_matrix * this->transformation_matrix;
    }

    std::vector<pcl::KdTreeFLANN<pcl::PointXYZRGB>> kdtrees(ref_clusters.size());
    for (size_t i = 0; i < ref_clusters.size(); i++)
        kdtrees[i].setInputCloud(subclouds[i].ref_point_cloud.makeShared());

    std::vector<int> point_div_index(this->point_cloud.size(), -1);
    for (size_t i = 0; i < this->point_cloud.size(); i++)
    {
        float min_dis = FLT_MAX;
        for (size_t j = 0; j < kdtrees.size(); j++)
        {
            std::vector<int> _idx_(1);
            std::vector<float> _dis_(1);
            kdtrees[j].nearestKSearch(point_cloud[i], 1, _idx_, _dis_);
            if (_dis_[0] < min_dis)
                point_div_index[i] = j;
        }
    }

    for (size_t i = 0; i < this->point_cloud.size(); i++)
        subclouds[point_div_index[i]].point_cloud.push_back(this->point_cloud[i]);

    for (size_t i = 0; i < subclouds.size(); i++)
    {
        subclouds[i].change_ref_point_cloud(ref_clusters[i]);
        cloud_transformation(subclouds[i].point_cloud, subclouds[i].transformation_matrix.inverse());
        subclouds[i].transformation_matrix = subclouds[i].transformation_matrix.inverse() * this->transformation_matrix;
    }

}

void cloud_transformation(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                 Eigen::Matrix4f matrix)
{
    for (size_t i = 0; i < cloud.size(); i++)
    {
        float new_x = matrix(0, 0) * cloud[i].x
                    + matrix(0, 1) * cloud[i].y
                    + matrix(0, 2) * cloud[i].z
                    + matrix(0, 3) * 1.0f;
        float new_y = matrix(1, 0) * cloud[i].x
                    + matrix(1, 1) * cloud[i].y
                    + matrix(1, 2) * cloud[i].z
                    + matrix(1, 3) * 1.0f;
        float new_z = matrix(2, 0) * cloud[i].x
                    + matrix(2, 1) * cloud[i].y
                    + matrix(2, 2) * cloud[i].z
                    + matrix(2, 3) * 1.0f;
        cloud[i].x = new_x, cloud[i].y = new_y, cloud[i].z = new_z;
    }
}

float local_icp(pcl::PointCloud<pcl::PointXYZRGB> &ref_cluster,
                pcl::PointCloud<pcl::PointXYZRGB> &point_cloud,
                Cloud &cloud)
{
    /* do localized icp */
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree, ref_tree;
    pcl::PointCloud<pcl::PointXYZRGB> result, matching_result;

    icp.setInputSource(ref_cluster.makeShared());
    icp.setInputTarget(point_cloud.makeShared());

//    ref_tree->setInputCloud(ref_cluster.makeShared());
//    tree->setInputCloud(point_cloud.makeShared());
//
//    icp.setSearchMethodSource(ref_tree);
//    icp.setSearchMethodTarget(tree);

    icp.setMaxCorrespondenceDistance(MAX_CORRESPONDENCE_DISTANCE);
    icp.setEuclideanFitnessEpsilon(MSE_DIFFERENCE_THRESHOLD);
    icp.setMaximumIterations(MAX_ICP_ITERATION);
    icp.setTransformationEpsilon(TRANSFORMATION_DIFFERENCE_THRESHOLD);

    icp.align(result);
    //pcl::io::savePLYFile("../test/data/" + std::to_string(all_index.size()) + ".ply", result);
    float mse = 0.0f;
    //Eigen::Matrix4f matrix = icp.getFinalTransformation();

    /* search point_cloud and add point whose distance from result is less than mse */
//    pcl::KdTreeFLANN<pcl::PointXYZRGB> result_tree;
//    result_tree.setInputCloud(result.makeShared());

//    for (size_t i = 0; i < point_cloud.size(); i++)
//    {
//        std::vector<int> _idx_(1);
//        std::vector<float> _dis_(1);
//        result_tree.nearestKSearch(point_cloud[i], 1, _idx_, _dis_);
////        if (_dis_[0] <= OVERLAP_SQRT_DISTANCE_THRESHOLD)
////        {
////            matching_result.push_back(point_cloud[i]);
////            all_index.insert(i);
////        }
//    }

//    /* inversely transform the selected points */
//    cloud_transformation(matching_result, matrix.inverse());
//
//    /* set them to cloud */
//    cloud.change_point_cloud(matching_result);
    cloud.point_cloud.clear();
    if (icp.hasConverged())
        cloud.change_ref_point_cloud(result),
        cloud.transformation_matrix = icp.getFinalTransformation(),
        mse = icp.getFitnessScore();
    else
    {
        cloud.change_ref_point_cloud(ref_cluster),
        cloud.transformation_matrix = Eigen::Matrix4f::Identity();
        int cnt = std::rand();
        pcl::io::savePLYFile(std::to_string(cnt) + ".ply", ref_cluster);
        pcl::io::savePLYFile(std::to_string(cnt) + "b.ply", point_cloud);
    }

    return mse;
}
