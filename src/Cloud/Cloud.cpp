#include "Cloud.h"

Cloud::Cloud()
{

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

    return;
}

float Cloud::total_base_icp()
{
    /* icp algorithm, tranform point_cloud to get max overlap with ref_point_cloud */
    pcl::PointCloud<pcl::PointXYZRGB> result;

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(this->point_cloud.makeShared());
    icp.setInputTarget(this->ref_point_cloud.makeShared());

    icp.setMaxCorrespondenceDistance(MAX_CORRESPONDENCE_DISTANCE);
    icp.setEuclideanFitnessEpsilon(TRANSFORMATION_DIFFERENCE_THRESHOLD);

    icp.align(result);
    Eigen::Matrix4f trans = icp.getFinalTransformation();

    this->point_cloud.swap(result);

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

    return ;
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
    return;
}

void Cloud::cluster_matching(std::vector<Cloud> &subclouds)
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> ref_clusters(*std::max_element(this->ref_cluster_index.begin(), this->ref_cluster_index.end()));
    pcl::PointCloud<pcl::PointXYZRGB> noise_cluster;
    for (size_t i = 0; i < this->ref_cluster_index.size(); i++)
    {
        if (this->ref_cluster_index[i] != 0)
            ref_clusters[this->ref_cluster_index[i] - 1].push_back(this->ref_point_cloud[i]);
        else
            noise_cluster.push_back(this->ref_point_cloud[i]);
    }

    for (size_t i = 0; i < ref_clusters.size(); i++)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        pcl::PointCloud<pcl::PointXYZRGB> result;

        icp.setInputSource(ref_clusters[i].makeShared());
        icp.setInputTarget(this->point_cloud.makeShared());

        icp.setMaxCorrespondenceDistance(MAX_CORRESPONDENCE_DISTANCE);
        icp.setEuclideanFitnessEpsilon(TRANSFORMATION_DIFFERENCE_THRESHOLD);

        icp.align(result);

        Eigen::Matrix4f trans = icp.getFinalTransformation();

    }
}