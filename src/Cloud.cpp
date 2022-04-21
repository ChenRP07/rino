#include "Cloud.h"

Cloud::Cloud()
{
    this->transformation_matrix = Eigen::Matrix4f::Identity();
}

int Cloud::set_point_cloud(const std::string& filename)
{
    if (pcl::io::loadPLYFile(filename, this->point_cloud) == -1)
    {
        std::cerr << "ERROR! Cannot load point cloud from file " << filename << "." << std::endl;
        return -1;
    }

    return 0;
}

int Cloud::set_ref_point_cloud(const std::string& filename)
{
    if (pcl::io::loadPLYFile(filename, this->ref_point_cloud) == -1)
    {
        std::cerr << "ERROR! Cannot load point cloud from file " << filename << "." << std::endl;
        return -1;
    }

    return 0;
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

    for (auto & i : this->point_cloud)
    {
        Cp.x += i.x;
        Cp.y += i.y;
        Cp.z += i.z;
    }

    for (auto & i : this->ref_point_cloud)
    {
        Cr.x += i.x;
        Cr.y += i.y;
        Cr.z += i.z;
    }

    Cp.x /= (float)this->point_cloud.size(), Cp.y /= (float)this->point_cloud.size(), Cp.z /= (float)this->point_cloud.size();
    Cr.x /= (float)this->ref_point_cloud.size(), Cr.y /= (float)this->ref_point_cloud.size(), Cr.z /= (float)this->ref_point_cloud.size();

    /* for each point, do a translation */
    for (auto & i : this->point_cloud)
    {
        i.x = i.x - Cp.x + Cr.x;
        i.y = i.y - Cp.y + Cr.y;
        i.z = i.z - Cp.z + Cr.z;
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

    return (float)icp.getFitnessScore();
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


float Cloud::cluster_matching(std::vector<Cloud> &subclouds, std::ofstream& outname)
{
    subclouds.clear();
    subclouds.resize(this->ref_clusters.size());
    /* use these clusters to do icp */
    for (size_t i = 0; i < this->ref_clusters.size(); i++)
    {
        float mse = this->local_icp((int)i, subclouds[i]);
        outname << mse << std::endl;
    }
    /* create a cloud and record every points' index */
    std::vector<int> ref_cluster_index;
    std::vector<int> point_div_index;
    pcl::PointCloud<pcl::PointXYZRGB> ref_cloud;
    for (size_t i = 0; i < this->ref_clusters.size(); i++)
        for (size_t j = 0; j < this->ref_clusters[i].size(); j++)
            ref_cloud.push_back(this->ref_clusters[i][j]), ref_cluster_index.push_back((int)i);

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(ref_cloud.makeShared());


    for (size_t i = 0; i < this->point_cloud.size(); i++)
    {
        std::vector<int> _idx_(1);
        std::vector<float> _dis_(1);
        kdtree.nearestKSearch(point_cloud[i], 1, _idx_, _dis_);
        point_div_index.push_back(ref_cluster_index[_idx_[0]]);
    }

    for (size_t i = 0; i < this->point_cloud.size(); i++)
        subclouds[point_div_index[i]].point_cloud.push_back(this->point_cloud[i]);

    for (size_t i = 0; i < subclouds.size(); i++)
    {
        subclouds[i].change_ref_point_cloud(this->ref_clusters[i]);
        cloud_transformation(subclouds[i].point_cloud, subclouds[i].transformation_matrix.inverse());
        subclouds[i].transformation_matrix = subclouds[i].transformation_matrix.inverse() * this->transformation_matrix;
    }
    return 1.0f;
}

void cloud_transformation(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                          Eigen::Matrix4f matrix)
{
    for (auto & i : cloud)
    {
        float new_x = matrix(0, 0) * i.x
                    + matrix(0, 1) * i.y
                    + matrix(0, 2) * i.z
                    + matrix(0, 3) * 1.0f;
        float new_y = matrix(1, 0) * i.x
                    + matrix(1, 1) * i.y
                    + matrix(1, 2) * i.z
                    + matrix(1, 3) * 1.0f;
        float new_z = matrix(2, 0) * i.x
                    + matrix(2, 1) * i.y
                    + matrix(2, 2) * i.z
                    + matrix(2, 3) * 1.0f;
        i.x = new_x, i.y = new_y, i.z = new_z;
    }
}

float Cloud::local_icp(int ref_cluster_idx, Cloud &cloud)
{
    if (this->ref_clusters[ref_cluster_idx].size() <= 10)
        return FLT_MAX;

    pcl::PointCloud<pcl::PointXYZRGB> align_cloud;
    align_cloud.resize(this->ref_clusters[ref_cluster_idx].size());

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(this->point_cloud.makeShared());
    Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();

    for (auto & i : this->ref_clusters[ref_cluster_idx])
    {
        std::vector<int> _idx_(1);
        std::vector<float> _dis_(1);
        kdtree.nearestKSearch(i, 1, _idx_, _dis_);
        translation(0, 3) += (this->point_cloud[_idx_[0]].x - i.x);
        translation(1, 3) += (this->point_cloud[_idx_[0]].y - i.y);
        translation(2, 3) += (this->point_cloud[_idx_[0]].z - i.z);
    }

    translation(0, 3) /= (float)this->ref_clusters[ref_cluster_idx].size();
    translation(1, 3) /= (float)this->ref_clusters[ref_cluster_idx].size();
    translation(2, 3) /= (float)this->ref_clusters[ref_cluster_idx].size();
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
    if (icp.hasConverged())
    {
        cloud.change_ref_point_cloud(result);
        cloud.transformation_matrix = icp.getFinalTransformation() * translation;
        auto mse = (float)icp.getFitnessScore();
        //std::cout << ref_cluster_idx << " " << mse << std::endl;
        return mse;
    }
    else
    {
        //pcl::io::savePLYFile("1.ply",this->ref_clusters[ref_cluster_idx]);
        return FLT_MAX;
    }
}
