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

#include <ros/ros.h>
#include <DynamicJoinPcl.h>
#include <NormalEstimationPcl.h>
#include <ColorNormalsPcl.h>
#include <ConversionPcl.h>
#include <MakeNormalsMarkers.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>

#include <robot_trajectory_saver_msgs/GetRobotTrajectories.h>

#include "KdTreeFLANN.h"


NormalEstimationPcl<pcl::PointXYZRGBNormal> normal_estimator;
ConversionPcl<pcl::PointXYZRGBNormal> conv_pcl;

ros::Publisher marker_normal_pub;
ros::Publisher pcl_pub;

ros::ServiceClient robot_traj_service_client;

std::string robot_frame_name;
std::string laser_frame_name;
std::string global_frame_name;
std::string robot_traj_service_name;

bool b_first_config = true;
bool b_first_time = true;

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
    T v;
    if (n.getParam(name, v))
    {
        ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
        return v;
    }
    else
        ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    return defaultValue;
}

void visualizeNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcl_norm)
{
    geometry_msgs::PoseArray poseArray;
    makeNormalsMarkers(pcl_norm, poseArray);
    ROS_INFO("normals poseArray size: %ld", poseArray.poses.size());
    marker_normal_pub.publish(poseArray);
}

void pointCloudCallback(const sensor_msgs::PointCloud2& pcl_msg)
{
    ROS_INFO("Received a new PointCloud2 message");

    NormalEstimationPclConfig normal_config = normal_estimator.getConfig();

    std::cout << "laser frame: " << normal_config.laser_frame << std::endl;
    std::cout << "num threads: " << normal_config.num_threads << std::endl;

    if (b_first_time)
    {
        b_first_time = false;

        if (robot_traj_service_client.waitForExistence(ros::Duration(5.0)))
        {
            robot_trajectory_saver_msgs::GetRobotTrajectories srv;
            if (robot_traj_service_client.call(srv))
            {
                //std::cout << "Number of robot poses red: " << srv.response.trajectories.poses.size() << std::endl;
                normal_estimator.setPrevMapTraj(srv.response.trajectories);
            }
            else
            {
                ROS_ERROR("Failed to call service robot trajectory");
            }
        }
        else
        {
            ROS_ERROR("Robot_traj_service is not UP!!! Check if there is any issue");
        }

    }

    // downsample
    pcl::PointCloud<pcl::PointXYZRGBNormal> pcl_in;
    pcl::PointCloud<pcl::PointXYZRGBNormal> pcl_out;
    conv_pcl.transform(pcl_msg, pcl_in);

    if (pcl_in.size() > 0)
    {
        /// < normal estimation
        pp::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
        kdtree.setInputCloud(pcl_in.makeShared());
        tf::Transform t;
        pcl::PointXYZ laser_center;
        conv_pcl.getLastTransform(t);
        conv_pcl.getFrameOrigin(normal_config.laser_frame, laser_center);
        normal_estimator.computeNormals(pcl_in, kdtree, laser_center);
        std::vector<int> index;
        pcl::removeNaNNormalsFromPointCloud(pcl_in, pcl_out, index);

        //colorNormalsPCL(map_pcl);
        visualizeNormals(pcl_out);
        sensor_msgs::PointCloud2 msg_out;
        pcl::toROSMsg(pcl_out, msg_out);
        pcl_pub.publish(msg_out);
    }
}


void savedTrajCallback(const nav_msgs::Path& saved_base_link_traj_msg)
{
    ROS_INFO("Received a saved base_link trajectory");
    normal_estimator.setPrevMapTraj(saved_base_link_traj_msg);
}

void normalConfigCallback(NormalEstimationPclConfig& config, uint32_t level)
{
    ROS_INFO("Received a new config message******************************************************************");

    // override the first message with the read parameter 
    if (b_first_config)
    {
        b_first_config = false;
        if (!laser_frame_name.empty())
        {
            config.laser_frame = laser_frame_name;
        }
    }
    normal_estimator.setConfig(config);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "compute_normals");

    tf::TransformListener tf_listener(ros::Duration(10.0));

    ros::NodeHandle n("~");

    /// < get parameters
    robot_frame_name = getParam<std::string>(n, "robot_frame_name", "/base_link");
    laser_frame_name = getParam<std::string>(n, "laser_frame_name", std::string());
    if (!laser_frame_name.empty())
    {
        NormalEstimationPclConfig& normal_config = normal_estimator.getConfig();
        normal_config.laser_frame = laser_frame_name;
    }
    robot_traj_service_name = getParam<std::string>(n, "robot_traj_service_name", "/robot_trajectory_saver_node/get_robot_trajectories_nav_msgs");

    /// < dynamic reconfigure 
    dynamic_reconfigure::Server<NormalEstimationPclConfig> normal_config_server(ros::NodeHandle("~/NormalEstimationPcl"));
    normal_config_server.setCallback(boost::bind(&normalConfigCallback, _1, _2));

    conv_pcl.setTFListener(tf_listener);

    /// < Input
    ros::Subscriber sub_pcl = n.subscribe("/cloud_in", 1, pointCloudCallback);
    ros::Subscriber sub_saved_trajectory = n.subscribe("/laser_mapper/trajectory_from_file", 1, savedTrajCallback);

    /// < Ouput
    marker_normal_pub = n.advertise<geometry_msgs::PoseArray>("/normals_marker", 1);
    pcl_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_out", 1, true);

    /// < Services 
    robot_traj_service_client = n.serviceClient<robot_trajectory_saver_msgs::GetRobotTrajectories>(robot_traj_service_name);

    ros::spin();
    return 0;
}

