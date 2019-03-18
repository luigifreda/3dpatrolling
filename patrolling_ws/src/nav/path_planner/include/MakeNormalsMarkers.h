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

#ifndef MAKE_NORMAL_MARKERS_H_
#define MAKE_NORMAL_MARKERS_H_

#include <cmath>
#include <vector>
#include <set>

#include <ros/ros.h>

#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <VoxelBinaryKey.h>

template<typename PointT>
void makeNormalsMarkers(const pcl::PointCloud<PointT>& pcl_norm, geometry_msgs::PoseArray& markers, double leaf_size = 0.1)
{
    // fill header
    markers.header.frame_id = pcl_norm.header.frame_id;
    markers.header.stamp = ros::Time::now();

    // X axis
    tf::Vector3 x = tf::Vector3(1, 0, 0);

    std::set<uint64_t> k;

    for (int i = 0; i < pcl_norm.size(); i++)
    {
        uint64_t ki = voxelBinaryKey(pcl_norm[i], leaf_size);
        if (k.count(ki)) continue;

        //if (!std::isnan(pcl_norm.points[i].normal_x) && !std::isnan(pcl_norm.points[i].normal_y) && !std::isnan(pcl_norm.points[i].normal_z) &&
        //        !std::isinf(pcl_norm.points[i].normal_x) && !std::isinf(pcl_norm.points[i].normal_y) && !std::isinf(pcl_norm.points[i].normal_z))
        if (std::isnormal(pcl_norm.points[i].normal_x) && std::isnormal(pcl_norm.points[i].normal_y) && std::isnormal(pcl_norm.points[i].normal_z) )
        {
            geometry_msgs::Pose pose;
            //tf::Vector3 v = tf::Vector3(pcl_norm.points[i].normal[0], pcl_norm.points[i].normal[1], pcl_norm.points[i].normal[2]);
            tf::Vector3 v = tf::Vector3(pcl_norm.points[i].normal_x, pcl_norm.points[i].normal_y, pcl_norm.points[i].normal_z);
            if (v.length2() > 1e-3)
            {
                tf::Vector3 cross = tf::tfCross(x, v);
                pose.position.x = pcl_norm.points[i].x;
                pose.position.y = pcl_norm.points[i].y;
                pose.position.z = pcl_norm.points[i].z;
                tf::Quaternion q = tf::Quaternion(cross.x(), cross.y(), cross.z(), sqrt(v.length2() * x.length2()) + tf::tfDot(x, v));
                q.normalize();
                tf::quaternionTFToMsg(q, pose.orientation);
                markers.poses.push_back(pose);
                k.insert(ki);
            }
        }
    }
}

#endif // MAKE_NORMAL_MARKERS_H_
