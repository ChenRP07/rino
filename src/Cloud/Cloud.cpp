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

void Cloud::change_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> &new_point_cloud)
{
    /* new cloud */
    this->point_cloud.resize(new_point_cloud.size());
    for (size_t i = 0; i < new_point_cloud.size(); i++)
        this->point_cloud[i] = new_point_cloud[i];
    /* init transformation matrix */
    this->transformation_matrix = Eigen::Matrix4f::Identity();
}

void Cloud::change_ref_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> &new_ref_point_cloud)
{
    this->ref_point_cloud.resize(new_ref_point_cloud.size());
    for (size_t i = 0; i < new_ref_point_cloud.size(); i++)
        this->ref_point_cloud[i] = new_ref_point_cloud[i];
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

void Cloud::constant_clustering()
{
    /* cluster centroids */
    pcl::PointCloud<pcl::PointXYZRGB> centroids;
    constant_cluster_centroid(this->ref_point_cloud, centroids);

    /* init */
    this->ref_clusters.resize(centroids.size());

    /* do the nearest search */
    pcl::KdTreeFLANN<pcl::PointXYZRGB> tree;
    tree.setInputCloud(centroids.makeShared());
    for (size_t i = 0; i < this->ref_point_cloud.size(); i++)
    {
        std::vector<int> _idx_(1);
        std::vector<float> _dis_(1);
        tree.nearestKSearch(this->ref_point_cloud[i], 1, _idx_, _dis_);
        /* add point to cluster according to its nearest neighbor in centroids */
        this->ref_clusters[_idx_[0]].push_back(ref_point_cloud[i]);
    }
}


void Cloud::cluster_matching(std::vector<Cloud> &subclouds)
{
    subclouds.clear();
    subclouds.resize(this->ref_clusters.size());

    /* use these clusters to do icp */
    for (size_t i = 0; i < this->ref_clusters.size(); i++)
    {
        this->local_icp(i, subclouds[i]);
    }

    std::vector<pcl::KdTreeFLANN<pcl::PointXYZRGB>> kdtrees(this->ref_clusters.size());
    for (size_t i = 0; i < this->ref_clusters.size(); i++)
        if (this->ref_clusters[i].size() != 0)
            kdtrees[i].setInputCloud(subclouds[i].ref_point_cloud.makeShared());

    std::vector<int> point_div_index(this->point_cloud.size(), -1);
    for (size_t i = 0; i < this->point_cloud.size(); i++)
    {
        float min_dis = FLT_MAX;
        for (size_t j = 0; j < kdtrees.size(); j++)
        {
            if (this->ref_clusters[j].size() != 0)
            {
                std::vector<int> _idx_(1);
                std::vector<float> _dis_(1);
                kdtrees[j].nearestKSearch(point_cloud[i], 1, _idx_, _dis_);
                if (_dis_[0] < min_dis)
                    point_div_index[i] = j, min_dis = _dis_[0];
            }
        }
    }

    for (size_t i = 0; i < this->point_cloud.size(); i++)
        subclouds[point_div_index[i]].point_cloud.push_back(this->point_cloud[i]);

    for (size_t i = 0; i < subclouds.size(); i++)
    {
        subclouds[i].change_ref_point_cloud(this->ref_clusters[i]);
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

float Cloud::local_icp(int ref_cluster_idx, Cloud &cloud)
{
    if (this->ref_clusters[ref_cluster_idx].size() == 0)
        return FLT_MAX;

    pcl::PointCloud<pcl::PointXYZRGB> align_cloud;
    align_cloud.resize(this->ref_clusters[ref_cluster_idx].size());

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(this->point_cloud.makeShared());
    Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();

    for (size_t i = 0; i < this->ref_clusters[ref_cluster_idx].size(); i++)
    {
        std::vector<int> _idx_(1);
        std::vector<float> _dis_(1);
        kdtree.nearestKSearch(this->ref_clusters[ref_cluster_idx][i], 1, _idx_, _dis_);
        translation(0, 3) += (this->point_cloud[_idx_[0]].x - this->ref_clusters[ref_cluster_idx][i].x);
        translation(1, 3) += (this->point_cloud[_idx_[0]].y - this->ref_clusters[ref_cluster_idx][i].y);
        translation(2, 3) += (this->point_cloud[_idx_[0]].z - this->ref_clusters[ref_cluster_idx][i].z);
    }

    translation(0, 3) /= this->ref_clusters[ref_cluster_idx].size();
    translation(1, 3) /= this->ref_clusters[ref_cluster_idx].size();
    translation(2, 3) /= this->ref_clusters[ref_cluster_idx].size();
    for (size_t i = 0; i < this->ref_clusters[ref_cluster_idx].size(); i++)
    {
        align_cloud[i] = this->ref_clusters[ref_cluster_idx][i];
        align_cloud[i].x += translation(0, 3);
        align_cloud[i].y += translation(1, 3);
        align_cloud[i].z += translation(2, 3);
    }

    /* do localized icp */
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    pcl::PointCloud<pcl::PointXYZRGB> result;

    icp.setInputSource(align_cloud.makeShared());
    icp.setInputTarget(point_cloud.makeShared());

    icp.setMaxCorrespondenceDistance(MAX_CORRESPONDENCE_DISTANCE);
    icp.setEuclideanFitnessEpsilon(MSE_DIFFERENCE_THRESHOLD);
    icp.setMaximumIterations(MAX_ICP_ITERATION);
    icp.setTransformationEpsilon(TRANSFORMATION_DIFFERENCE_THRESHOLD);

    icp.align(result);

    cloud.point_cloud.clear();
    cloud.change_ref_point_cloud(result);
    cloud.transformation_matrix = icp.getFinalTransformation() * translation;
    float mse = icp.getFitnessScore();
    //std::cout << ref_cluster_idx << " " << mse << std::endl;
    return mse;
}
