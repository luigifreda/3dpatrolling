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

#ifndef TRAJECTORY_CONTROL_ACTIONSERVER_H_
#define TRAJECTORY_CONTROL_ACTIONSERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <trajectory_control_msgs/TrajectoryControlAction.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nifti_robot_driver_msgs/Tracks.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include <trajectory_control_msgs/PlanningFeedback.h>
#include <trajectory_control_msgs/PlanningTask.h>
#include <trajectory_control_msgs/MultiRobotPath.h> 
#include <trajectory_control_msgs/message_enums.h> // for SegmentStatus and TaskType

#include <boost/thread/recursive_mutex.hpp>
#include <boost/shared_ptr.hpp>

#include "SignalUtils.h"
#include "PathManager.h"
#include "LowPassFilter.h"
#include "CmdVels.h"

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
    {
        ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    }
    return defaultValue;
}

///	\class TrajectoryControlActionServer
///	\author Luigi Freda (2016-Present) and Alcor (2016)
///	\brief Trajectory control node
///	\note 
/// 	\todo 
///	\date
///	\warning
class TrajectoryControlActionServer
{
    
public:

    enum ControlLawType
    {
        kInputOutputFL = 0, 
        kNonLinear
    }; 

    
public:
    
    static const double kTrackDistance; // [m] distance between tracks 
    
    static const float kRobotRadius;  // [m] robot radius for computing the clearance  
    static const float kProximityDistanceThreshold; // [m] distance point-to-robot 
    static const float kProximityActiveAngle; // [rad]
        
    static const double kDefaulRefVelocity; // [m/s] default linear velocity 
    static const double kDefaultDistanceOffsetPointB; // [m] distance offset of point B from robot center (this is used in the control law) 
                                                // position of point B from robot pose (x,y,theta)
                                                // xB = x + displacement * cos(theta)
                                                // yB = y + displacement * sin(theta)
    static const double kLaserProximityReducedVelocityFactor; // [frac %] reducing factor for linear velocity when laser proximity is detected 
    static const double kLaserProximityReducedVelocityResetTime; // [s] if elapsed time from last laser proximity check is > kLaserProximityReducedVelocityResetTime, the normal velocity is set back 
    
    static const double kDefaultControlGainK1_IOL; // default control gain k1 
    static const double kDefaultControlGainK2_IOL; // default control gain k2
    static const double kDefaultControlGainK1_NL; // default control gain k1 
    static const double kDefaultControlGainK2_NL; // default control gain k2
    static const double kDefaultAngularGainKw;  // default control angular gain kw: a pure rotatation control is applied if the angular error is above kRefAngularErrorForPureRotationControl
    static const double kRefAngularErrorForPureRotationControl; 
    static const double kRefXYErrorForPureRotationControl; 
    
    static const double kAngularErrorThreshold;    
    
    static const int kDefaultControlLaw; // default control law (should be one in the ControlLawType list)
    static const double kDefaultControlFrequency; // [Hz] default control frequency 
    static const double kMaxTrackVelocity; // [m/s] maximum track velocity 
    static const double kMaxTrackVelocityOnRotation; //[m/s]  maximum track velocity in m/s on rotations 
    static const double kMaxAngularVelocity; //[rad/s]  maximum angular velocity 

    
    
    //static const double kMinimumWaitingTimeforANewPath; // [s] minimum time to wait for giving a new path as input 
    
    static const double kTimeOutTolerance; // [s], this time will be added to the expected trajectory execution time in order to define a time out
    
    static const double kAdpativeTravFreq; // [Hz] default refresh frequency for adaptive traversability velocity
    static const double kAdpativeTravFilterWn; // [rad/s] default natural frequency for the adaptive traversability velocity filter  
    
    static const double kRefErrorForStoppingPathManagerStep; // max reference error for allowing path manager to step  
    //static const double k3DDistanceArrivedToGoal; // [m]
    //static const double k2DDistanceArrivedToGoal; // [m]
    
    
    static const std::string kTeleopMuxPriorityName; 
    
protected:
    
    ros::NodeHandle node_;
    ros::NodeHandle param_node_;
    
    std::string action_name;
    actionlib::SimpleActionServer<trajectory_control_msgs::TrajectoryControlAction> act_server_;
    actionlib::SimpleActionClient<trajectory_control_msgs::TrajectoryControlAction> act_client_;

    trajectory_control_msgs::TrajectoryControlFeedback feedback_msg_;
    trajectory_control_msgs::TrajectoryControlResult result_msg_;

    std::string odom_frame_id_;
    std::string global_frame_id_;
    std::string robot_frame_id_;

    std::string fl_frame_id_;
    std::string fr_frame_id_;
    std::string rl_frame_id_;
    std::string rr_frame_id_;
    std::string imu_frame_id_;

    //std::vector<double> tip_over_axes_coeffs_;
    std::vector<tf::Vector3> tip_over_axes_vecs_;

    tf::TransformListener tf_listener_;

    std::string imu_odom_topic_;
    ros::Subscriber imu_odom_sub_;

    std::string tracks_vel_cmd_topic_;
    ros::Publisher tracks_vel_cmd_pub_;

    tf::StampedTransform tf_robot_pose_map_;
    tf::StampedTransform tf_robot_pose_odom_;
    tf::StampedTransform tf_odom_to_map_;
    tf::StampedTransform tf_robot_poseB_map_; // point B with offset
    boost::recursive_mutex tf_robot_pose_map_mutex;

    tf::StampedTransform tf_front_left_flipper_;
    tf::StampedTransform tf_front_right_flipper;
    tf::StampedTransform tf_rear_left_flipper_;
    tf::StampedTransform tf_rear_right_flipper_;
    tf::StampedTransform tf_imu_t;

    double displacement_; // axis displacement of poin B (feedback linearization with offset)
    volatile double vel_reference_;
    double linear_vel_;
    double angular_vel_;
    double robot_width_;
    double k1_IOL_; // for input-output linearization control law
    double k2_IOL_; // for input-output linearization control law
    double k1_NL_; // for non linear control law
    double k2_NL_; // for non linear control law 
    double kw_; // gain for pure rotational control 
    double control_frequency_;
    ControlLawType control_law_type_;
    
    double vel_max_tracks_; // maximum allowed track velocity

    std::string global_path_topic_;
    ros::Publisher global_path_pub_;

    std::string local_path_topic_;
    ros::Publisher local_path_pub_;
    
    std::string teleop_mux_service_acquire_name_;
    std::string teleop_mux_service_release_name_;
    ros::ServiceClient teleop_mux_service_acquire_;
    ros::ServiceClient teleop_mux_service_release_;
    ros::Publisher teleop_mux_pub_;
    bool b_use_teleop_mux_service_;   // does it exist the service? 

    nav_msgs::Path global_path_msg_;
    nav_msgs::Path local_path_msg_;
    nav_msgs::Path global_plan_msg_;

    std::string cmd_topic_;
    ros::Publisher cmd_pub_;

    std::string imu_topic_;
    ros::Subscriber imu_sub_;

    std::string robot_path_topic_;
    ros::Subscriber robot_path_sub_;
    
    std::string robot_local_path_topic_;
    ros::Subscriber robot_loal_path_sub_;
    
    std::string robot_rotation_topic_;
    ros::Subscriber robot_rotation_sub_;    
    
    std::string goal_abort_topic_;
    ros::Subscriber goal_abort_sub_;
    
    std::string trajectory_control_abort_topic_;
    ros::Subscriber trajectory_control_abort_sub_;
    
    std::string multi_robot_paths_topic_; 
    ros::Publisher multi_robot_paths_pub_; 
    
    boost::shared_ptr<PathManager> p_path_manager_; 
    //PathManagerKdt p_path_manager_;
    
    boost::recursive_mutex action_mutex;    
    
public:   /// <  laser proximity checker integration 
        
    std::string laser_proximity_topic_; 
    ros::Subscriber laser_proximity_sub_; // subscriber for the laser proximity checker on/off message
    
    std::string closest_obst_point_topic_; 
    ros::Subscriber closest_obst_point_sub_; // subscriber for the closer obstacle point
    
    std::string closest_obst_vel_reduction_enable_topic_; 
    ros::Subscriber closest_obst_vel_reduction_enable_sub_; // subscriber for the enabling/disabling velocity reduction depending on closer obstacle point
    
    boost::recursive_mutex closest_obst_point_mutex;
    tf::Vector3 closest_obst_point;
    double closest_obst_dist; 
    
    bool b_closest_obst_vel_reduction_enable_;
    
    volatile bool b_decreased_vel_; // for decreasing the velocity when the robot is close to an obstacle
    ros::Time decrease_vel_activation_time_; // time the last laser proximity msg was received 
    double velocity_before_reduction_; 
        
public:   /// < adpative traversability stuff 
    
    std::string adapt_trav_vel_topic_; 
    ros::Subscriber adapt_trav_vel_sub_; // subscriber for maximum velocity coming from adaptive traversability module
    double adapt_trav_vel_; 
    LowPassFilter filter_vel_trav_; 
    double adaptive_trav_vel_freq_; // refresh frequency 
    
protected: /// < queue controller stuff 
    
    ros::Publisher queue_task_feedback_pub_; // to say "hey, I'm ready!"
    ros::Subscriber queue_task_feedback_sub_; // to know when to stop
    ros::Subscriber queue_task_path_sub_; // path to follow (global path planning)
    ros::Subscriber queue_task_path_local_sub_; // path to follow (local path planning)
    
    std::string queue_task_feedback_topic_;
    std::string queue_task_path_topic_; 
    
    std::string queue_task_path_local_topic_; 
    
    volatile bool b_local_path_;
    volatile bool b_simple_rotation_;
    
    int robot_id_; 
    std::string str_robot_name_;

public:
    
    TrajectoryControlActionServer(std::string);
    ~TrajectoryControlActionServer();
    
    // convert and odo msg to stamped transform 
    void odomMsgToStampedTransform(nav_msgs::Odometry pose_odometry_msg, tf::StampedTransform& pose_stamped_tf);
    
    /// < compute the position of point B from robot pose (x,y,theta)
    // xB = x + displacement * cos(theta)
    // yB = y + displacement * sin(theta)
    void getRealRobotPoseB(double displacement, tf::StampedTransform real_robot_pose, tf::StampedTransform& real_robot_poseB);
    
    // two methods for getting current useful transforms
    // robot_pose_odom, robot_pose_map, from_odom_to_map
    bool getRobotPose(tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&);
    // robot_pose_odom, robot_pose_map, from_odom_to_map, frontLeftF,frontRightF, rearLeftF, rearRightF, imu_t
    bool getRobotPose2(tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&);
    
    // resample the trajectory in order to have a point for each delta = vel*Ts
    void resampleAndSmoothPath(const nav_msgs::Path&, nav_msgs::Path&);
    
    // compute ref signals on point B starting from ref signals on robot center 
    void buildReferenceTrajectory(double, const geometry_msgs::PoseStamped&, geometry_msgs::Pose&, geometry_msgs::Twist&);
    
    // compute control law and return the current error
    double computeControlLawIOLin(const tf::StampedTransform& tf_robot_pose_B, double k1, double k2, const geometry_msgs::Pose& ref_pose_B, const geometry_msgs::Twist& ref_vel_B, double& linear_vel, double& angular_vel);
    double computeControlLawPosition(const tf::StampedTransform& tf_robot_pose_B, double k1, double k2, const geometry_msgs::PoseStamped& target_pose, double& linear_vel, double& angular_vel);
    double computeControlLawNonLin(const tf::StampedTransform& tf_robot_pose, double k1, double k2, const geometry_msgs::Pose& ref_pose, const geometry_msgs::Twist& ref_vel, double& linear_vel, double& angular_vel);
    double computeControlLawPosition2(const tf::StampedTransform& tf_robot_pose_B, double k1, double k2, const geometry_msgs::PoseStamped& target_pose, double& linear_vel, double& angular_vel);
    
    double computeControlLawRotation(const tf::StampedTransform& tf_robot_pose, const geometry_msgs::Pose& ref_pose, const geometry_msgs::Twist& ref_vel, double& linear_vel, double& angular_vel);

    // compute and saturate track commands 
    void getTracksVelCmd(double, double, double, nifti_robot_driver_msgs::Tracks&);
    
public: /// < callbacks     
    
    void imuOdomCallback(const nav_msgs::OdometryConstPtr&);
    void imuDataCallback(const sensor_msgs::ImuConstPtr&);
        
    void executeCallback(const trajectory_control_msgs::TrajectoryControlGoalConstPtr&);
    
    void robotPathCallBack(const nav_msgs::PathConstPtr&);
    void robotLocalPathCallBack(const nav_msgs::PathConstPtr&);
    
    void robotRotationCallBack(const nav_msgs::PathConstPtr&);
    
    void adaptTravVelCallback(const geometry_msgs::TwistStampedPtr&); 
        
    void queueFeedbackCallback(const trajectory_control_msgs::PlanningFeedback& feedback_msg);
    void queueTaskCallback(const nav_msgs::Path& path_msg);
    void queueLocalTaskCallback(const nav_msgs::Path& path_msg);
    
    void goalAbortCallback(const std_msgs::Bool& msg); 
    
    void laserProximityCallback(const std_msgs::Bool& msg); 
    
    void closestObstaclePointCallback(const std_msgs::Float32MultiArray& msg); 
    
    void closestObstacleVelReductionEnableCallback(const std_msgs::Bool& msg);
    
protected: /// < other functions          
    
    void executeRotation(const trajectory_control_msgs::TrajectoryControlGoalConstPtr&);
    
    void checkLaserProximityAndUpdateVelocity();
    
    void tipOverAxis(tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, std::vector<double>&);
    void tipOverAxis(tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, tf::StampedTransform&, std::vector<tf::Vector3>&);
    
    void sendMultiRobotPath(const nav_msgs::Path& path_msg); 
    
    void sendVelCommands(const nifti_robot_driver_msgs::Tracks& tracks_cmd, double lin_vel, double angular_vel); 
    void sendCommandsToNiftiTelopMux(double lin_vel, double angular_vel);

    void acquireTeleopMux();
    void releaseTeleopMux();
};


#endif /* TRAJECTORYCONTROLACTIONSERVER_H_ */
