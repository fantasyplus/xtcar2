#ifndef HYBRID_ASTAR_PLANNER_ROS_H
#define HYBRID_ASTAR_PLANNER_ROS_H

// std C++
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <thread>

// ros
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>

//自定义
#include "high_performence_hybrid_astar_ros.hpp"

namespace hybrid_astar_planner_ros
{
  class HybridAstarPlannerRos : public nav_core::BaseGlobalPlanner
  {
  public:
    HybridAstarPlannerRos();
    HybridAstarPlannerRos(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    ~HybridAstarPlannerRos();

  public:
    /*---------------------move_base 调用接口---------------------*/
    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped &start,
                  const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan) override;

  private:
    /*---------------------ros相关---------------------*/
    ros::Publisher _pub_initial_path;
    ros::Publisher _pub_smoothed_path;

    bool _initialized;

  private:
    /*---------------------hybrid A*相关---------------------*/
    HybridAstar _hybrid_astar;
    PlannerCommonParam _hybrid_astar_param;

    inline bool isSuccess(const SearchStatus &status);
    // void setCostMap(costmap_2d::Costmap2DROS *costmap_ros);

    void costmapCallback(const nav_msgs::OccupancyGrid &msg);
    bool _costmap_initialized;
    ros::Subscriber _costmap_sub;
    nav_msgs::OccupancyGrid _occupancy_grid;

  private:
    /*---------------------tf变换相关---------------------*/
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

    geometry_msgs::TransformStamped getTransform(const string &target, const string &source);

    nav_msgs::Path transferTrajectory(const geometry_msgs::Pose &current_pose,
                                      const TrajectoryWaypoints &trajectory_waypoints);
    geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose,
                                      const geometry_msgs::TransformStamped &transform);

  private:
    /*---------------------用于可视化最终轨迹---------------------*/
    ros::Publisher _pub_path_vehicles;
    visualization_msgs::MarkerArray _path_vehicles; //车子数据结构，用于可视化

    struct color
    {
      float red;
      float green;
      float blue;
    };

    static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};

    static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};

    static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};

    static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};

    static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};

    static constexpr color black = {0.f / 255.f, 0.f / 255.f, 0.f / 255.f};

    static constexpr color blue = {51.f / 255.f, 204.f / 255.f, 204.f / 255.f};

    void visualPathVehicle(const TrajectoryWaypoints &visual_waypoints);
  };
}

#endif
