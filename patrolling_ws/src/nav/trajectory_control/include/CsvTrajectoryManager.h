/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DPATROLLING. 
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

#ifndef CSV_TRAJECTORY_MANAGER_H
#define CSV_TRAJECTORY_MANAGER_H

#include <boost/shared_ptr.hpp>

#include "PathManager.h"
#include "CSVReader.h"


///	\class CsvTrajectoryManager
///	\author Luigi Freda
///	\brief Class for reading a trajectory from cvs data 
///	\note
/// 	\todo 
///	\date
///	\warning

class CsvTrajectoryManager : public BasePathManager
{
public:

    CsvTrajectoryManager() : i_traj_length_(0)
    {
        path_type_ = BasePathManager::kPathFollowing;
    }

    CsvTrajectoryManager(const std::string& csv_file, double Ts)
    {
        init(csv_file, Ts);
    }

    ~CsvTrajectoryManager()
    {
    }

    /// Init

    void init(const std::string& csv_file, double Ts)
    {
        path_type_ = BasePathManager::kTrajectoryTracking;
        
        b_init_ = true;
        b_end_  = false;

        d_Ts_ = Ts;
        p_csv_reader_.reset(new CSVReader(csv_file));

        std::vector< std::vector<double> >& mat_data = p_csv_reader_->getData();

        i_traj_length_ = mat_data.size();

        for (int i = 0; i < i_traj_length_; i++)
        {
            //labels: x_d y_d theta_d v_d omega_d xp_d yp_d xpp_d ypp_d
            //idx:      0   1       2   3       4    5    6     7     8
            x_.push_back(mat_data[i][0]);
            y_.push_back(mat_data[i][1]);
            theta_.push_back(mat_data[i][2]);
            v_.push_back(mat_data[i][3]);
            omega_.push_back(mat_data[i][4]);
            xp_.push_back(mat_data[i][5]);
            yp_.push_back(mat_data[i][6]);
            xpp_.push_back(mat_data[i][7]);
            ypp_.push_back(mat_data[i][8]);
        }

        initPoses();
    }

    inline void initPoses();

    inline void addTrajectoryOffset(double x0, double y0);

    /// basic step for generating next ref point 
    /// return true when done 

    bool step()
    {
        if (!b_init_)
        {
            ROS_ERROR("CsvTrajectoryManager::Step() - you did not init!");
            b_end_ = true;

            return false; /// < EXIT POINT 
        }

        // save previous pose 
        prev2_pose_ = prev_pose_;
        prev_pose_  = current_pose_;

        if (i_index_ < i_traj_length_)
        {
            current_pose_.pose.position.x = x_[i_index_]; // start from first point 
            current_pose_.pose.position.y = y_[i_index_];
            current_pose_.pose.position.z = 0;
            current_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(theta_[i_index_]);

            d_current_yaw_ = theta_[i_index_];

            d_vel_lin_ = v_[i_index_];
            d_vel_ang_ = omega_[i_index_];
        }
        else
        {
            b_end_ = true;
        }


        i_index_++;
        return b_end_;
    }
    
    inline void print();

public: // setters 

public: // getters

protected:

    boost::shared_ptr<CSVReader> p_csv_reader_;

    // current assumed label sequence x_d y_d theta_d v_d omega_d xp_d yp_d xpp_d ypp_d
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> theta_;
    std::vector<double> v_;
    std::vector<double> omega_;
    std::vector<double> xp_; // dot x
    std::vector<double> yp_; // dot y
    std::vector<double> xpp_; // ddot x
    std::vector<double> ypp_; // ddot y

    int i_traj_length_;

};

void CsvTrajectoryManager::initPoses()
{
    current_pose_ = geometry_msgs::PoseStamped();
    final_pose_ = geometry_msgs::PoseStamped();

    //current_pose_.header = path_in_.poses[0].header;
    current_pose_.pose.position.x = x_[0]; // start from first point 
    current_pose_.pose.position.y = y_[0];
    current_pose_.pose.position.z = 0;
    current_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(theta_[0]);

    // for init 
    prev_pose_  = current_pose_;
    prev2_pose_ = current_pose_;

    final_pose_.pose.position.x = x_[i_traj_length_ - 1]; // start from first point 
    final_pose_.pose.position.y = y_[i_traj_length_ - 1];
    final_pose_.pose.position.z = 0;
    final_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(theta_[i_traj_length_ - 1]);

    d_vel_lin_ = v_[0];
    d_vel_ang_ = omega_[0];

    //    size_t input_path_length = path_in_.poses.size();
    //    final_pose_.header = path_in_.poses[input_path_length - 1].header;
    //    final_pose_.pose.position = path_in_.poses[input_path_length - 1].pose.position;
    //    final_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    //i_index_ = 1; /// < start from second point, the current_ one has been stored in current_

    d_estimated_time_     = 0;
    d_estimated_distance_ = 0;
    for (int ii = 0; ii < (i_traj_length_ - 2); ii++)
    {
        d_estimated_distance_ += sqrt(pow(x_[ii + 1] - x_[ii], 2) +
                                      pow(y_[ii + 1] - y_[ii], 2));
    }
    //d_estimated_time_ = ((fabs(d_vel_lin_) > std::numeric_limits<double>::epsilon()) ? d_estimated_distance_ / d_vel_lin_ : 0);
    d_estimated_time_ = d_Ts_ * i_traj_length_;


    //ROS_INFO("PathManager::init() - #points: %ld, estimated time: %f, estimated distance: %f", path_in_.poses.size(), d_estimated_time_, d_estimated_distance_);
}

void CsvTrajectoryManager::addTrajectoryOffset(double x0, double y0)
{
    for (int i = 0; i < i_traj_length_; i++)
    {
        x_[i] += x0;
        y_[i] += y0;
    }
}

void CsvTrajectoryManager::print()
{
    for (int i = 0; i < i_traj_length_; i++)
    {
        std::cout << x_[i] <<", " << y_[i] << ", " << theta_[i] 
                << ", " << v_[i] << ", " << omega_[i] <<", " << xp_[i] <<", "<< yp_[i] <<", "<< xpp_[i] <<", "<< ypp_[i] << std::endl;        
    }
}


#endif
