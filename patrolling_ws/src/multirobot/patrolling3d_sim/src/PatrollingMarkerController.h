#ifndef PATROLLING_MARKER_CONTROLLER_H_
#define PATROLLING_MARKER_CONTROLLER_H_

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>
#include <tf/tf.h>

#include <cstring>
#include <boost/thread/recursive_mutex.hpp>

#include <path_planner/Transform.h>

#include "ColorUtils.h"


class PatrollingMarkerController
{
    
    static const std::string kDefaultMakerName; 
    static const std::string kDefaultMakerDescription; 
    static const float kInitialVerticalOffset; 
    static const float kTextMessageHeight; 
    static const float kTextMessageVerticalOffset; 
    static const float kPoseUpdatePeriod; 
    
public:
    
    ros::NodeHandle nh_;
    
    visualization_msgs::InteractiveMarker int_marker_;
    ros::Publisher patrolling_pause_pub_;
    
    ros::Timer pose_timer_;    

    boost::recursive_mutex marker_server_mtx; 
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
    interactive_markers::MenuHandler menu_handler_;

    std::string marker_name_; 
    std::string ugv_name_; 
    std::string robot_frame_id_; 
    std::string world_frame_id_; 
        
    Transform transform_robot_; 
    
public: 
    PatrollingMarkerController(const tf::Vector3& p0, 
                     const ros::NodeHandle& nh,
                     const std::string& ugv_name = "ugv1",
                     const std::string& robot_frame_id = "/base_link",
                     const std::string& world_frame_id = "/map",
                     const std::string& patrol_pause_topic_name = "/patrolling/task/pause", 
                     const std::string& int_marker_server_name = "patrolling_marker_controller",
                     const std::string& marker_name = kDefaultMakerName); 
    ~PatrollingMarkerController();
    
    visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker&, const Color& color = Colors::LightGrey()  );
    visualization_msgs::InteractiveMarkerControl& makeBoxControl(visualization_msgs::InteractiveMarker&);
    void makeViewFacingMarker();
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
    void reset();
    
    void setMarkerPosition(const geometry_msgs::Pose &pose); 
    void setMarkerColor(const Color& color, const std::string& text = std::string()); 
    
    void updateRobotPosition(const ros::TimerEvent& timer_msg);
    
};


#endif //MARKER_CONTROLLER
