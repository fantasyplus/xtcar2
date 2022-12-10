#include "hybrid_astar_planner_ros.h"
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner_ros::HybridAstarPlannerRos, nav_core::BaseGlobalPlanner)

namespace hybrid_astar_planner_ros
{
  geometry_msgs::Pose HybridAstarPlannerRos::transformPose(const geometry_msgs::Pose &pose,
                                                           const geometry_msgs::TransformStamped &transform)
  {
    geometry_msgs::PoseStamped transformed_pose;
    geometry_msgs::PoseStamped orig_pose;
    orig_pose.pose = pose;
    tf2::doTransform(orig_pose, transformed_pose, transform);

    return transformed_pose.pose;
  }

  geometry_msgs::TransformStamped HybridAstarPlannerRos::getTransform(const string &target,
                                                                      const string &source)
  {
    geometry_msgs::TransformStamped tf;
    try
    {
      // _tf_buffer->setUsingDedicatedThread(true);
      tf = _tf_buffer->lookupTransform(target, source, ros::Time(0), ros::Duration(1));
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    return tf;
  }

  HybridAstarPlannerRos::HybridAstarPlannerRos()
      : _initialized(false), _costmap_initialized(false)
  {
    /*---------------------subscribe---------------------*/
    ros::NodeHandle _nh("");
    _costmap_sub = _nh.subscribe("move_base/global_costmap/costmap", 1, &HybridAstarPlannerRos::costmapCallback, this);
  }

  HybridAstarPlannerRos::HybridAstarPlannerRos(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
      : _initialized(false), _costmap_initialized(false)
  {
    initialize(name, costmap_ros);
  }

  HybridAstarPlannerRos::~HybridAstarPlannerRos()
  {
  }

  void HybridAstarPlannerRos::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {

    if (!_initialized)
    {
      _hybrid_astar.setNodeHandle(name);

      ros::NodeHandle _private_nh("~/" + name);

      //混合A*参数
      _private_nh.param<float>("time_limit", _hybrid_astar_param.time_limit, 10000.0);

      _private_nh.param<float>("vehicle_length", _hybrid_astar_param.vehiche_shape.length, 3.452);
      _private_nh.param<float>("vehicle_width", _hybrid_astar_param.vehiche_shape.width, 1.8);
      _private_nh.param<float>("vehicle_cg2back", _hybrid_astar_param.vehiche_shape.cg2back, 1.326);

      _private_nh.param<float>("max_turning_radius", _hybrid_astar_param.max_turning_radius, 20.0);
      _private_nh.param<float>("min_turning_radius", _hybrid_astar_param.min_turning_radius, 3.925);
      _private_nh.param<int>("turning_radius_size", _hybrid_astar_param.turning_radius_size, 1);
      _private_nh.param<int>("theta_size", _hybrid_astar_param.theta_size, 32);

      _private_nh.param<float>("reverse_weight", _hybrid_astar_param.reverse_weight, 4.0);
      _private_nh.param<float>("turning_weight", _hybrid_astar_param.turning_weight, 2.2);
      _private_nh.param<float>("goal_lateral_tolerance", _hybrid_astar_param.goal_lateral_tolerance, 1.0);
      _private_nh.param<float>("goal_longitudinal_tolerance", _hybrid_astar_param.goal_longitudinal_tolerance, 1.0);
      _private_nh.param<float>("goal_angular_tolerance", _hybrid_astar_param.goal_angular_tolerance, 0.15708);
      _private_nh.param<int>("obstacle_threshold", _hybrid_astar_param.obstacle_threshold, 100);

      _private_nh.param<bool>("use_back", _hybrid_astar_param.use_back, true);
      _private_nh.param<bool>("use_reeds_shepp", _hybrid_astar_param.use_reeds_shepp, true);
      _private_nh.param<bool>("use_obstacle_heuristic", _hybrid_astar_param.use_obstacle_heuristic, false);
      _private_nh.param<bool>("use_analytic_expansion", _hybrid_astar_param.use_analytic_expansion, true);
      _private_nh.param<bool>("use_theta_cost", _hybrid_astar_param.use_theta_cost, false);
      _private_nh.param<float>("obstacle_theta_ratio", _hybrid_astar_param.obstacle_theta_ratio, 1.0);

      _private_nh.param<bool>("use_smoother", _hybrid_astar_param.use_smoother, false);
      _private_nh.param<bool>("use_map_inflated", _hybrid_astar_param.use_map_inflated, false);

      _private_nh.param<float>("alpha", _hybrid_astar_param.alpha, 0.1);
      _private_nh.param<float>("obstacle_weight", _hybrid_astar_param.obstacle_weight, 0.1);
      _private_nh.param<float>("curvature_weight", _hybrid_astar_param.curvature_weight, 0.0);
      _private_nh.param<float>("smoothness_weight", _hybrid_astar_param.smoothness_weight, 0.1);
      _private_nh.param<float>("obstacle_distance_max", _hybrid_astar_param.obstacle_distance_max, 1.0);

      _private_nh.param<float>("analytic_expansion_ratio", _hybrid_astar_param.analytic_expansion_ratio, 35);
      _private_nh.param<float>("analytic_expansion_max_length", _hybrid_astar_param.analytic_expansion_max_length, 30);

      _hybrid_astar.initParam(_hybrid_astar_param);

      /*---------------------advertise---------------------*/
      _pub_initial_path = _private_nh.advertise<nav_msgs::Path>("initial_path", 1, true);
      _pub_smoothed_path = _private_nh.advertise<nav_msgs::Path>("smoothed_path", 1, true);
      _pub_path_vehicles = _private_nh.advertise<visualization_msgs::MarkerArray>("path_vehicle", 1, true);

      _tf_buffer = std::make_shared<tf2_ros::Buffer>();
      _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

      _initialized = true;
      ROS_INFO("HybridAstarPlannerRos: initialized");
    }
    else
    {
      ROS_WARN("HybridAstarPlannerRos:planner has already been initialized... doing nothing");
    }
  }

  void HybridAstarPlannerRos::costmapCallback(const nav_msgs::OccupancyGrid &msg)
  {
    if (!_costmap_initialized)
    {
      _occupancy_grid = msg;

      _costmap_initialized = true;

      _hybrid_astar.SetOccupancyGrid(_occupancy_grid);
    }
  }

  // void HybridAstarPlannerRos::setCostMap(costmap_2d::Costmap2DROS *costmap_ros)
  // {
  //   _occupancy_grid = std::make_unique<nav_msgs::OccupancyGrid>();

  //   _occupancy_grid->header.frame_id = costmap_ros->getGlobalFrameID();
  //   _occupancy_grid->header.stamp = ros::Time::now();

  //   costmap_2d::Costmap2D *temp_costmap2d = costmap_ros->getCostmap();
  //   uint32_t width = temp_costmap2d->getSizeInCellsX();
  //   uint32_t height = temp_costmap2d->getSizeInCellsY();
  //   float resloution = temp_costmap2d->getResolution();

  //   _occupancy_grid->info.width = width - resloution / 2.0;
  //   _occupancy_grid->info.height = height - resloution / 2.0;
  //   _occupancy_grid->info.origin.position.x = temp_costmap2d->getOriginX();
  //   _occupancy_grid->info.origin.position.y = temp_costmap2d->getOriginY();
  //   _occupancy_grid->info.origin.position.z = 0.0;
  //   _occupancy_grid->info.origin.orientation.w = 1.0;
  //   _occupancy_grid->info.resolution = resloution;

  //   char *cost_translation_table_ = NULL;
  //   if (cost_translation_table_ == NULL)
  //   {
  //     cost_translation_table_ = new char[256];

  //     // special values:
  //     cost_translation_table_[0] = 0;     // NO obstacle
  //     cost_translation_table_[253] = 99;  // INSCRIBED obstacle
  //     cost_translation_table_[254] = 100; // LETHAL obstacle
  //     cost_translation_table_[255] = -1;  // UNKNOWN

  //     // regular cost values scale the range 1 to 252 (inclusive) to fit
  //     // into 1 to 98 (inclusive).
  //     for (int i = 1; i < 253; i++)
  //     {
  //       cost_translation_table_[i] = char(1 + (97 * (i - 1)) / 251);
  //     }
  //   }

  //   unsigned char *bin_map = temp_costmap2d->getCharMap();
  //   _occupancy_grid->data.resize(width * height);
  //   for (unsigned int i = 0; i < _occupancy_grid->data.size(); i++)
  //   {
  //     _occupancy_grid->data[i] = cost_translation_table_[bin_map[i]];
  //   }
  //   delete[] cost_translation_table_;

  //   _hybrid_astar.SetOccupancyGrid(*_occupancy_grid);
  // }

  bool HybridAstarPlannerRos::makePlan(const geometry_msgs::PoseStamped &start,
                                       const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
  {
    const auto start_pose_in_costmap_frame = transformPose(start.pose,
                                                           getTransform(_occupancy_grid.header.frame_id, start.header.frame_id));
    ROS_INFO("HybridAstarPlannerRos:start_pose_in_costmap_frame:%f,%f", start_pose_in_costmap_frame.position.x, start_pose_in_costmap_frame.position.y);

    const auto goal_pose_in_costmap_frame = transformPose(goal.pose,
                                                          getTransform(_occupancy_grid.header.frame_id, goal.header.frame_id));
    ROS_INFO("HybridAstarPlannerRos:goal_pose_in_costmap_frame:%f,%f", goal_pose_in_costmap_frame.position.x, goal_pose_in_costmap_frame.position.y);

    TimerClock t0;
    SearchStatus result = _hybrid_astar.makePlan(start_pose_in_costmap_frame, goal_pose_in_costmap_frame);
    ROS_INFO("HybridAstarPlannerRos:Hybrid Astar planning: %f [ms]", t0.getTimerMilliSec());

    if (isSuccess(result))
    {
      ROS_INFO("HybridAstarPlannerRos:Plan found.");
      TrajectoryWaypoints waypoints = _hybrid_astar.getTrajectory();

      _hybrid_astar.visualAnalyticPath();

      // _hybrid_astar.visualCollisionClear();
      visualPathVehicle(waypoints);
      _hybrid_astar.visualOpenNode();

      //初始路径
      nav_msgs::Path initial_traj = transferTrajectory(start.pose, waypoints);
      _pub_initial_path.publish(initial_traj);
      plan = initial_traj.poses;

      //平滑后的路径
      if (_hybrid_astar_param.use_smoother)
      {
        TimerClock t1;
        _hybrid_astar.smoothPath(waypoints);
        ROS_INFO("HybridAstarPlannerRos:smooth path cost: %f [ms]", t1.getTimerMilliSec());

        nav_msgs::Path smooth_traj = transferTrajectory(start.pose, waypoints);
        _pub_smoothed_path.publish(smooth_traj);
      }

      //计算重置耗时
      TimerClock t2;
      _hybrid_astar.reset();
      ROS_INFO("HybridAstarPlannerRos:reset cost: %f [ms]", t2.getTimerMilliSec());

      return true;
    }
    else
    {
      switch (result)
      {
      case SearchStatus::FAILURE_COLLISION_AT_START:
        ROS_INFO("HybridAstarPlannerRos:Cannot find plan because collision was detected in start position.");
        break;
      case SearchStatus::FAILURE_COLLISION_AT_GOAL:
        ROS_INFO("HybridAstarPlannerRos:Cannot find plan because collision was detected in goal position.");
        break;
      case SearchStatus::FAILURE_TIMEOUT_EXCEEDED:
        ROS_INFO("HybridAstarPlannerRos:Cannot find plan because timeout exceeded.");
        break;
      case SearchStatus::FAILURE_NO_PATH_FOUND:
        ROS_INFO("HybridAstarPlannerRos:Cannot find plan.");
        break;
      default:
        ROS_INFO("HybridAstarPlannerRos:SearchStatus not handled.");
        break;
      }
      return false;
    }
  }

  inline bool HybridAstarPlannerRos::isSuccess(const SearchStatus &status)
  {
    return status == SearchStatus::SUCCESS;
  }

  nav_msgs::Path HybridAstarPlannerRos::transferTrajectory(const geometry_msgs::Pose &current_pose,
                                                           const TrajectoryWaypoints &trajectory_waypoints)
  {
    nav_msgs::Path ros_traj;
    ros_traj.header = trajectory_waypoints.header;

    for (auto single_wp : trajectory_waypoints.trajectory)
    {
      geometry_msgs::PoseStamped temp_pose;

      temp_pose.header = trajectory_waypoints.header;

      temp_pose.pose.position.x = single_wp.pose.pose.position.x;
      temp_pose.pose.position.y = single_wp.pose.pose.position.y;
      temp_pose.pose.position.z = single_wp.pose.pose.position.z;

      temp_pose.pose.orientation = single_wp.pose.pose.orientation;

      ros_traj.poses.push_back(temp_pose);
    }

    return ros_traj;
  }

  void HybridAstarPlannerRos::visualPathVehicle(const TrajectoryWaypoints &visual_waypoints)
  {
    int clear_index = 0;
    static int id = 0;
    if (clear_index == 0)
    {
      visualization_msgs::Marker vehicle_marker;
      vehicle_marker.header.frame_id = "map";
      vehicle_marker.header.stamp = ros::Time::now();
      vehicle_marker.id = id++;
      vehicle_marker.action = 3;

      _path_vehicles.markers.push_back(vehicle_marker);

      clear_index = 1;
    }

    for (int i = 0; i < visual_waypoints.trajectory.size(); i++)
    {
      visualization_msgs::Marker vehicle_marker;

      vehicle_marker.header.frame_id = "map";
      vehicle_marker.header.stamp = ros::Time::now();
      vehicle_marker.id = id++;
      vehicle_marker.type = visualization_msgs::Marker::CUBE;

      vehicle_marker.scale.x = _hybrid_astar_param.vehiche_shape.length;
      vehicle_marker.scale.y = _hybrid_astar_param.vehiche_shape.width;
      vehicle_marker.scale.z = 1.0;

      vehicle_marker.color.a = 0.2;

      if (i != visual_waypoints.trajectory.size() - 1 &&
          visual_waypoints.trajectory[i].is_back != visual_waypoints.trajectory[i + 1].is_back)
      {
        vehicle_marker.color.r = blue.red;
        vehicle_marker.color.g = blue.green;
        vehicle_marker.color.b = blue.blue;
      }
      else if (visual_waypoints.trajectory[i].is_back == false)
      {
        vehicle_marker.color.r = green.red;
        vehicle_marker.color.g = green.green;
        vehicle_marker.color.b = green.blue;
      }
      else if (visual_waypoints.trajectory[i].is_back == true)
      {
        vehicle_marker.color.r = pink.red;
        vehicle_marker.color.g = pink.green;
        vehicle_marker.color.b = pink.blue;
      }

      const auto vis_pose_in_mapframe = transformPose(visual_waypoints.trajectory[i].pose.pose,
                                                      getTransform("map", _occupancy_grid.header.frame_id));

      vehicle_marker.pose = vis_pose_in_mapframe;
      _path_vehicles.markers.push_back(vehicle_marker);
    }

    _pub_path_vehicles.publish(_path_vehicles);
  }
}