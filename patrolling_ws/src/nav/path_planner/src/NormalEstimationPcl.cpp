/**
* This file is part of the ROS package path_planner which belongs to the framework 3DPATROLLING. 
*
* Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> and Alcor Lab (La Sapienza University)
* For more information see <https://gitlab.com/luigifreda/3dpatrolling>
*
* 3DPATROLLING is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DPATROLLING is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DPATROLLING. If not, see <http://www.gnu.org/licenses/>.
*/

#include <NormalEstimationPcl.h>
#include <ZTimer.h>

#define NORM3_L2(x, y, z) (sqrt(pow((x), 2) + pow((y), 2) + pow((z), 2)))

#ifdef LOG_TIMES
#define LOG_TIME(obj, name, ...) (obj).measureAndLog(name, __VA_ARGS__) 
#else
#define LOG_TIME(obj, name, ...)
#endif

template<typename PointT>
const float NormalEstimationPcl<PointT>::kLaserZOffset = 0.4; // [m] how much each laser view point is pushed higher for the pseudo-normals evaluation
template<typename PointT>
const float NormalEstimationPcl<PointT>::kLaserZWrtBody = 0.2; // [m] actual delta_z between laser view point and base_link
template<typename PointT>
const int NormalEstimationPcl<PointT>::kMinNumNeighboursForComputingNormal = 5; // minimum number of neighbours for computing the normal 


template<typename PointT>
NormalEstimationPcl<PointT>::NormalEstimationPcl() :
threshold_(0.5)
{
}

template<typename PointT>
NormalEstimationPcl<PointT>::~NormalEstimationPcl()
{
}

template<typename PointT>
inline float NormalEstimationPcl<PointT>::kernel(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
{
    switch (config_.kernel_type)
    {
    case NormalEstimationPcl_Gaussian: return gaussianKernel(v1, v2);
    case NormalEstimationPcl_Cosine: return cosineKernel(v1, v2);
    default: return 0;
    }
}

template<typename PointT>
inline float NormalEstimationPcl<PointT>::gaussianKernel(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
{
    //Implement the standard gaussian kernel: k(v1,v2) = exp(-||v1-v2||^2 / h)
    return exp(-pow((v1 - v2).norm(), 2) / config_.smoothing);
}

template<typename PointT>
inline float NormalEstimationPcl<PointT>::cosineKernel(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
{
    return pow((1 + cos(M_PI * (v1 - v2).norm() / config_.radius))*0.5, config_.smoothing);
}

template<typename PointT>
void NormalEstimationPcl<PointT>::computeCovarianceMatrix(const PointCloudT& neighbors, const pcl::PointXYZ& center, Eigen::Matrix3f& covariance_matrix)
{
    const Eigen::Vector3f c(center.x, center.y, center.z);
    covariance_matrix.setZero();
    for (size_t i = 0, iEnd = neighbors.size(); i < iEnd; i++)
    {
        const Eigen::Vector3f v(neighbors[i].x, neighbors[i].y, neighbors[i].z);
        const Eigen::Vector3f diff = v - c;
        covariance_matrix += kernel(v, c) * (diff * diff.transpose());
    }
    covariance_matrix /= neighbors.size();
}

template<typename PointT>
bool NormalEstimationPcl<PointT>::computeNormal(const size_t i, const size_t begin, const size_t end, PointCloudT& pcl, KdTreeT& kdtree, std::vector<bool>& done, const pcl::PointXYZ& center)
{
    if (done[i]) return true;

    //Eigen::Vector3f pseudonormal(Eigen::Vector3f(center.x, center.y, center.z) - Eigen::Vector3f(pcl[i].x, pcl[i].y, pcl[i].z));
    
    const size_t center_idx = closest_laser_point_idx_[i];
    const Eigen::Vector3f laser_center(pcl_laser_trajectory_[center_idx].x, pcl_laser_trajectory_[center_idx].y, pcl_laser_trajectory_[center_idx].z);
    Eigen::Vector3f pseudonormal( laser_center - Eigen::Vector3f(pcl[i].x, pcl[i].y, pcl[i].z));    
    pseudonormal.normalize();

#ifndef USE_PSEUDONORMALS_AS_NORMALS
    //Find nighbors of point
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if (kdtree.radiusSearch(pcl[i], config_.radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > kMinNumNeighboursForComputingNormal)
    {

#if 0
        /* if there are too many neighbors, try to downsample at random: */
        size_t old_size = pointIdxRadiusSearch.size(), num_removed = 0;
        while (pointIdxRadiusSearch.size() > 10)
        {
            size_t index = rand() % pointIdxRadiusSearch.size();
            //std::swap(*(pointIdxRadiusSearch.begin() + index), pointIdxRadiusSearch.back());
            //pointIdxRadiusSearch.pop_back();
            //std::swap(*(pointRadiusSquaredDistance.begin() + index), pointRadiusSquaredDistance.back());
            //pointRadiusSquaredDistance.pop_back();
            pointIdxRadiusSearch.erase(pointIdxRadiusSearch.begin() + index);
            pointRadiusSquaredDistance.erase(pointRadiusSquaredDistance.begin() + index);
            num_removed++;
        }
        if (num_removed)
        {
            ROS_INFO("Random downsampling removed %d points (initial neighbor count was %d)", num_removed, old_size);
        }
#endif


        ZTimer ztimer;

        //If neighbors are more than 5 compute the covariance_matrix
        //cout<<"neghbors size:"<<pointIdxRadiusSearch.size()<<endl;

        PointCloudT neighbors;
        pcl::PointXYZ baricenter(0.0, 0.0, 0.0);
        size_t size = 0;
        for (size_t j = 0, jEnd = pointIdxRadiusSearch.size(); j < jEnd; j++)
        {
            const PointT& point_j = pcl[pointIdxRadiusSearch[j]];
            neighbors.push_back(point_j);
            baricenter.x += point_j.x;
            baricenter.y += point_j.y;
            baricenter.z += point_j.z;
            size++;
        }
        baricenter.x /= size;
        baricenter.y /= size;
        baricenter.z /= size;

        LOG_TIME(ztimer, "build_neighbours", size);

        typedef Eigen::Matrix3f MatrixT;

        MatrixT covariance_matrix;
        computeCovarianceMatrix(neighbors, baricenter, covariance_matrix);

        LOG_TIME(ztimer, "covariance_matrix", size);

        //
        //		//From the covariance matrix compute the normal using PCA with Eigeinvalue decomposition
        //		Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix, true);
        //
        //
        //
        //		float e0 = es.eigenvalues()(0).real();
        //		float e1 = es.eigenvalues()(1).real();
        //		float e2 = es.eigenvalues()(2).real();
        //
        //
        //
        //		float min_e = std::min(e0,e1);
        //		min_e = std::min(min_e,e2);
        //		Vector3cf v1c,v2c;
        //		if (min_e==e0){
        //			v1c = es.eigenvectors().col(1);
        //			v2c = es.eigenvectors().col(2);
        //		}
        //		else if(min_e==e1){
        //			v1c = es.eigenvectors().col(0);
        //			v2c = es.eigenvectors().col(2);
        //		}
        //		else{
        //			v1c = es.eigenvectors().col(0);
        //			v2c = es.eigenvectors().col(1);
        //		}

        //Eigen::JacobiSVD<MatrixT> svd(covariance_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::JacobiSVD<MatrixT> svd(covariance_matrix, Eigen::ComputeFullV);
        Eigen::Vector3f Sv = svd.singularValues();
        MatrixT T = covariance_matrix * svd.matrixV().transpose();
        const float e0 = T.col(0).norm();
        const float e1 = T.col(1).norm();
        const float e2 = T.col(2).norm();
        float min_e = std::min( std::min(e0, e1), e2);        
        Eigen::Vector3f v1c, v2c;
        if (min_e == e0)
        {
            v1c = T.col(1) / e1;
            v2c = T.col(2) / e2;
        }
        else if (min_e == e1)
        {

            v1c = T.col(0) / e0;
            v2c = T.col(2) / e2;
        }
        else
        {
            v1c = T.col(1) / e1;
            v2c = T.col(0) / e0;
        }

        //		//Take just the real part
        //		std::complex<float> x = vc(0);
        //		std::complex<float> y = vc(1);
        //		std::complex<float> z = vc(2);
        //		Eigen::Vector3f normal(x.real(),y.real(),z.real());

        //Take just the real part of both the eigenvectors
        const std::complex<float> x1 = v1c(0);
        const std::complex<float> y1 = v1c(1);
        const std::complex<float> z1 = v1c(2);
        const Eigen::Vector3f v1(x1.real(), y1.real(), z1.real());
        const std::complex<float> x2 = v2c(0);
        const std::complex<float> y2 = v2c(1);
        const std::complex<float> z2 = v2c(2);
        const Eigen::Vector3f v2(x2.real(), y2.real(), z2.real());

        //Compute the cross product
        Eigen::Vector3f normal = v1.cross(v2);
        //cout <<"normla "<<i<<": "<<"-"<<normal[0]<<"-"<<normal[1]<<"-"<<normal[2]<<endl;
#if 0
        if (normal.norm() < 1e-3)
        {
            ROS_INFO("NormalEstimationPcl::computeNormal: point %d: abnormal normal! ", i);
            ROS_INFO("NormalEstimationPcl::computeNormal:     e0=%f, e1=%f, e2=%f", e0, e1, e2);
            ROS_INFO("NormalEstimationPcl::computeNormal:     normal: %f, %f, %f", normal(0), normal(1), normal(2));
        }
#endif

        normal.normalize();
        if (normal.dot(pseudonormal) < 0)
        {
            normal = -1 * normal;
        }

        pcl[i].normal[0] = normal(0);
        pcl[i].normal[1] = normal(1);
        pcl[i].normal[2] = normal(2);
        done[i] = true;

        LOG_TIME(ztimer, "svd", size);

        
        if (3 * Sv[2] / (Sv[0] + Sv[1] + Sv[2]) < config_.flatness_curvature_threshold)
        {
            //std::cout<<"find flat aerea"<<std::endl;
            for (size_t j = 0, jEnd = pointIdxRadiusSearch.size(); j < jEnd; j++)
            {
                pcl::PointXYZ delta;
                delta.x = baricenter.x - pcl[pointIdxRadiusSearch[j]].x;
                delta.y = baricenter.y - pcl[pointIdxRadiusSearch[j]].y;
                delta.z = baricenter.z - pcl[pointIdxRadiusSearch[j]].z;
                if (pointIdxRadiusSearch[j] >= begin
                        && pointIdxRadiusSearch[j] <= end
                        && NORM3_L2(delta.x, delta.y, delta.z) < config_.radius * 0.5)
                {
                    pcl[pointIdxRadiusSearch[j]].normal[0] = normal(0);
                    pcl[pointIdxRadiusSearch[j]].normal[1] = normal(1);
                    pcl[pointIdxRadiusSearch[j]].normal[2] = normal(2);
                    done[pointIdxRadiusSearch[j]] = true;
                }
            }
        }

        LOG_TIME(ztimer, "block_assign");

        return true;
    }
    else
    {
        /// < TODO: interpolate instead of assigning NAN
        
        pcl[i].normal[0] = NAN;
        pcl[i].normal[1] = NAN;
        pcl[i].normal[2] = NAN;
        done[i] = true;

        //cout << "NormalEstimationPcl::computeNormal: point " << i << ": isolated point will have no normal " << endl;

        return false;
    }
#else
    pcl[i].normal[0] = pseudonormal(0);
    pcl[i].normal[1] = pseudonormal(1);
    pcl[i].normal[2] = pseudonormal(2);
    done[i] = true;
#endif
}

template<typename PointT>
void NormalEstimationPcl<PointT>::computeNormalsInRange(const size_t num_thread, const size_t start, const size_t end, PointCloudT& pcl, KdTreeT& kdtree, std::vector<bool>& done, const pcl::PointXYZ& center)
{
    ROS_INFO("NormalEstimationPcl::computeNormalsInRange: spawn thread #%ld (point range: %ld-%ld)", num_thread, start, end);

    try
    {
        size_t num_computed_points = 0, num_isolated_points = 0, num_invalid_normals = 0;
        ZTimer ztimer;

        for (size_t i = start; i < end; i++)
        {
            if (!done[i])
            {
                num_computed_points++;
            }

            if (!computeNormal(i, start, end, pcl, kdtree, done, center))
            {
                num_isolated_points++;
            }

            if (NORM3_L2(pcl[i].normal_x, pcl[i].normal_y, pcl[i].normal_z) < 0.99)
            {
                num_invalid_normals++;
            }
        }

        double tt = ztimer.measure();
        ROS_INFO("NormalEstimationPcl::computeNormalsInRange: finished thread #%ld: computed %ld normals in %fs (%f normal/s), #isolated: %ld, #invalid: %ld", num_thread, num_computed_points, tt, num_computed_points / tt, num_isolated_points, num_invalid_normals);
    }
    catch (boost::thread_interrupted&)
    {
        ROS_INFO("NormalEstimationPcl::computeNormalsInRange: thread #%ld has been interrupted", num_thread);
    }
}

template<typename PointT>
void NormalEstimationPcl<PointT>::computeNormals(PointCloudT& pcl, KdTreeT& kdtree, const pcl::PointXYZ& center)
{
    addPointToLaserTraj(center);
    
    kdtree_laser_centers_.setInputCloud(pcl_laser_trajectory_.makeShared());
        
    ros::Time time_start = ros::Time::now();
    computeClosestLaserPoints(pcl);
    ros::Duration elapsed_time = ros::Time::now() - time_start;
    ROS_INFO_STREAM("computeClosestLaserPoints - elapsed time: " << elapsed_time); 
    
    boost::recursive_mutex::scoped_lock locker(pcl_laser_traj_mutex_);
    
    const size_t n = pcl.size();
    if (n > 0)
    {
        //cout << "NormalEstimationPcl::computeNormals: begin (pcl size: " << n << ")" << endl;
        //		for (int i = 0; i < input_pcl.size(); i++) {
        //			pcl::PointXYZRGBNormal p;
        //			if (compute_normal(input_pcl.points[i], p)){
        //				if(!std::isnan(p.normal_x) && !std::isnan(p.normal_y) && !std::isnan(p.normal_z) &&
        //						!std::isinf(p.normal_x) && !std::isinf(p.normal_y) && !std::isinf(p.normal_z)){
        //					output.push_back(p);
        //				}
        //			}
        //		}

        //std::vector<bool> done;
        //for (int i = 0; i < pcl.size(); i++)
        //{
        //    done.push_back(false);
        //}
        std::vector<bool> done( pcl.size(),false); 
        
        if (config_.num_threads > 1)
        {
            std::vector<boost::shared_ptr<boost::thread> > threads;
            for (size_t num_thread = 0; num_thread < config_.num_threads; num_thread++)
            {
                size_t begin = n * num_thread / config_.num_threads;
                size_t end   = n * (num_thread + 1) / config_.num_threads;
                threads.push_back(
                                  boost::make_shared<boost::thread>(
                                  boost::bind(
                                              &NormalEstimationPcl::computeNormalsInRange,
                                              this,
                                              num_thread,
                                              begin,
                                              end,
                                              boost::ref(pcl),
                                              boost::ref(kdtree),
                                              boost::ref(done),
                                              boost::cref(center)
                                              )
                                  )
                                  );
            }

            for (size_t num_thread = 0; num_thread < threads.size(); num_thread++)
            {
                threads[num_thread]->join();
            }
        }
        else
        {
            computeNormalsInRange(0, 0, n, pcl, kdtree, done, center);
        }


        size_t num_done = 0;
        for (size_t i = 0, iEnd = done.size(); i < iEnd; i++)
        {
            if (done[i]) num_done++;
        }
        double percent_done = (double) (100 * num_done) / (double) done.size();
        ROS_INFO("NormalEstimationPcl::computeNormals: done: %f%%", percent_done);

        size_t num_invalid_normals = 0;
        for (size_t i = 0, iEnd = pcl.size(); i < iEnd; i++)
        {
            if (NORM3_L2(pcl[i].normal_x, pcl[i].normal_y, pcl[i].normal_z) < 0.99)
            {
                num_invalid_normals++;
            }
        }

        ROS_INFO("NormalEstimationPcl::computeNormals: finished; # invalid normals: %ld", num_invalid_normals);
    }
    else
    {
        ROS_WARN("NormalEstimationPcl::computeNormals: received empty point cloud");
    }
}

template<typename PointT>
void NormalEstimationPcl<PointT>::addPointToLaserTraj(const pcl::PointXYZ& center)
{
    boost::recursive_mutex::scoped_lock locker(pcl_laser_traj_mutex_);
    
    PointT center_point; 
    center_point.x = center.x; 
    center_point.y = center.y;
    center_point.z = center.z + kLaserZOffset;
    pcl_laser_trajectory_.push_back(center_point);
}


// Compute closest laser point to each point of the cloud 
template<typename PointT>
void NormalEstimationPcl<PointT>::computeClosestLaserPoints(PointCloudT& pcl)
{
    boost::recursive_mutex::scoped_lock locker(pcl_laser_traj_mutex_);
        
    closest_laser_point_idx_.resize(pcl.size()); 
    
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    
    for(size_t jj=0, jjEnd = pcl.size(); jj < jjEnd; jj++)
    {
        // k-nearest neighborhood 
        int found = kdtree_laser_centers_.nearestKSearch(pcl[jj], 1, pointIdxNKNSearch, pointNKNSquaredDistance);   
        if(found<1)
        {
            ROS_WARN("NormalEstimationPcl<PointT>::computeClosestLaserPoints() - cannot find a close node "); 
            // get the last laser pose as reference
            closest_laser_point_idx_[jj] = pcl_laser_trajectory_.size()-1;
        }
        else
        {
            closest_laser_point_idx_[jj] = pointIdxNKNSearch[0];
        }
    }
    
}

// Set previous map base_link trajectory 
template<typename PointT>
void NormalEstimationPcl<PointT>::setPrevMapTraj(const nav_msgs::Path& base_link_traj)
{
    boost::recursive_mutex::scoped_lock locker(pcl_laser_traj_mutex_);
    
    // reset map trajectory 
    pcl_prev_map_trajectory_.clear();
    
    PointT center_point; 
    
    int path_length = base_link_traj.poses.size();
    for (int ii = 0; ii < path_length; ii++)
    {
        /// < approx: here we compute the laser centers by only offsetting the z coordinate
        center_point.x = base_link_traj.poses[ii].pose.position.x; 
        center_point.y = base_link_traj.poses[ii].pose.position.y; 
        center_point.z = base_link_traj.poses[ii].pose.position.z + kLaserZWrtBody; 
        
        std::cout << "NormalEstimationPcl::setPrevMapTraj() - p("<<ii<<"): " <<  base_link_traj.poses[ii].pose.position << std::endl; 
    
        pcl_prev_map_trajectory_.push_back(center_point); 
    }
    
    pcl_laser_trajectory_ += pcl_prev_map_trajectory_; 
}


// Set laser trajectory 
template<typename PointT>
void NormalEstimationPcl<PointT>::setLaserTraj(const nav_msgs::Path& base_link_traj)
{
    boost::recursive_mutex::scoped_lock locker(pcl_laser_traj_mutex_);
    
    // reset laser trajectory 
    pcl_laser_trajectory_.clear();
    
    PointT center_point; 
    
    const int path_length = base_link_traj.poses.size();
    for (int ii = 0; ii < path_length; ii++)
    {
        /// < approx: here we compute the laser centers by only offsetting the z coordinate
        center_point.x = base_link_traj.poses[ii].pose.position.x; 
        center_point.y = base_link_traj.poses[ii].pose.position.y; 
        center_point.z = base_link_traj.poses[ii].pose.position.z + kLaserZWrtBody;
    
        pcl_laser_trajectory_.push_back(center_point); 
    }
    
    pcl_laser_trajectory_ += pcl_prev_map_trajectory_; 
}
    
    
template class NormalEstimationPcl<pcl::PointNormal>;
template class NormalEstimationPcl<pcl::PointXYZINormal>;
template class NormalEstimationPcl<pcl::PointXYZRGBNormal>;
