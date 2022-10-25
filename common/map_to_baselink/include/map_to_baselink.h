#ifndef M2B_H
#define M2B_h

// std c++
#include <iostream>
#include <string>

// ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

//gazebo
#include <gazebo_msgs/GetModelState.h>

class M2B
{
public:
    M2B();

private:
    ros::NodeHandle _nh;
    ros::NodeHandle _private_nh;

    ros::ServiceClient _srv_get_pose;

    ros::Timer _timer_tf;

private:
    std::string map_frame, base_link_frame;

    nav_msgs::Odometry _base_pose_ground_truth;

private:
    void getPose();
    
private:
    tf::TransformBroadcaster _tf_broadcaster;
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    geometry_msgs::TransformStamped _target_tf;

    void callbackTimerPublishTF(const ros::TimerEvent &e);
    void publishTF();
    geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose,
                                      const geometry_msgs::TransformStamped &transform);

    geometry_msgs::TransformStamped getTransform(const std::string &target,
                                                 const std::string &source);
};

#endif