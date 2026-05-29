-- ================================================================
-- Cartographer 2D SLAM 配置文件
-- 适用于差速驱动机器人 + 360度激光雷达
-- 参考: https://google-cartographer-ros2.readthedocs.io/
-- ================================================================

-- 包含 Cartographer 核心库的默认配置
include "map_builder.lua"
include "trajectory_builder.lua"

-- ================================================================
-- 全局选项
-- ================================================================
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- 坐标系配置
  map_frame = "map",              -- 地图坐标系
  tracking_frame = "base_link",   -- 跟踪坐标系 (机器人基座)
  published_frame = "odom",       -- 已发布的坐标系 (里程计)
  odom_frame = "odom",            -- 里程计坐标系

  -- TF 发布配置
  provide_odom_frame = false,     -- 不发布 odom→base_link TF (由 Gazebo 差速插件发布)
  publish_frame_projected_to_2d = true,  -- 将 TF 投影到 2D 平面
  publish_tracked_pose = true,    -- 发布跟踪位姿

  -- 使用里程计数据
  use_odometry = true,
  use_nav_sat = false,            -- 不使用 GPS
  use_landmarks = false,          -- 不使用路标

  -- 激光雷达配置
  num_laser_scans = 1,            -- 单线激光雷达数量
  num_multi_echo_laser_scans = 0, -- 不使用多回波激光
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,           -- 不使用 3D 点云
}

-- ================================================================
-- 2D 位姿图选项 (Pose Graph)
-- 用于回环检测和全局优化
-- ================================================================
MAP_BUILDER.use_trajectory_2d = true

-- 后端优化选项
POSE_GRAPH.optimization_problem = {
  huber_scale = 1e1,           -- Huber 损失函数的尺度
  acceleration_weight = 1e1,   -- 加速度权重
  rotation_weight = 1e2,       -- 旋转权重
  local_slam_pose_translation_weight = 1e5,
  local_slam_pose_rotation_weight = 1e5,
  odometry_translation_weight = 1e5,
  odometry_rotation_weight = 1e5,
  fixed_frame_pose_translation_weight = 1e1,
  fixed_frame_pose_rotation_weight = 1e1,
  log_solver_summary = false,
  use_online_imu_extrinsics_in_3d = true,
  fix_z_in_3d = false,
  ceres_solver_options = {
    use_nonmonotonic_steps = false,
    max_num_iterations = 50,
    num_threads = 1,
  },
}

-- 全局约束构建器 (回环检测)
POSE_GRAPH.constraint_builder = {
  sampling_ratio = 0.3,                  -- 采样率
  max_constraint_distance = 15.0,        -- 最大约束距离 (米)
  min_score = 0.55,                      -- 最小匹配分数
  global_localization_min_score = 0.6,   -- 全局定位最小分数
  loop_closure_translation_weight = 1.1e4,
  loop_closure_rotation_weight = 1e5,
  log_matches = true,
  fast_correlative_scan_matcher = {
    linear_search_window = 7.0,
    angular_search_window = math.rad(30.0),
    branch_and_bound_depth = 7,
  },
  ceres_scan_matcher = {
    occupied_space_weight = 20.0,
    translation_weight = 10.0,
    rotation_weight = 1.0,
    ceres_solver_options = {
      use_nonmonotonic_steps = true,
      max_num_iterations = 10,
      num_threads = 1,
    },
  },
}

-- 位姿图优化频率
POSE_GRAPH.optimize_every_n_nodes = 90    -- 每 90 个节点优化一次

-- 全局约束搜索
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.max_num_final_iterations = 200
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.0

-- ================================================================
-- 2D 轨迹构建器选项 (Trajectory Builder)
-- 负责局部 SLAM (前端)
-- ================================================================

-- 使用 2D 轨迹构建器
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 不使用 IMU (Gazebo 仿真中无 IMU)

-- 激光雷达数据处理
TRAJECTORY_BUILDER_2D.min_range = 0.1       -- 最小扫描距离 (米)
TRAJECTORY_BUILDER_2D.max_range = 5.0       -- 最大扫描距离 (米)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 累积的激光扫描数

-- 体素滤波器 (降采样)
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025  -- 体素滤波器尺寸 (米)

-- 自适应体素滤波器
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter = {
  max_length = 0.5,              -- 最大体素长度
  min_num_points = 150,          -- 最少点数
  max_range = 5.0,               -- 最大范围
}

-- Cere 扫描匹配器 (局部优化)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher = {
  occupied_space_weight = 10.0,
  translation_weight = 10.0,
  rotation_weight = 40.0,
  ceres_solver_options = {
    use_nonmonotonic_steps = false,
    max_num_iterations = 20,
    num_threads = 1,
  },
}

-- 实时相关性扫描匹配器 (前端快速匹配)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher = {
  linear_search_window = 0.1,
  angular_search_window = math.rad(20.0),
  translation_delta_cost_weight = 1e-1,
  rotation_delta_cost_weight = 1e-1,
}

-- 运动滤波器 (避免在静止时添加数据)
TRAJECTORY_BUILDER_2D.motion_filter = {
  max_time_seconds = 5.0,         -- 最大时间间隔 (秒)
  max_distance_meters = 0.2,      -- 最大距离间隔 (米)
  max_angle_radians = math.rad(1.0),  -- 最大角度间隔 (弧度)
}

-- 子图选项
TRAJECTORY_BUILDER_2D.submaps = {
  num_range_data = 90,                    -- 每个子图的激光扫描数
  grid_options_2d = {
    grid_type = "PROBABILITY_GRID",       -- 概率栅格地图
    resolution = 0.05,                    -- 地图分辨率 (米/像素)
  },
  range_data_inserter = {
    range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
    insert_free_space = true,
    hit_probability = 0.55,
    miss_probability = 0.49,
  },
}

-- ================================================================
-- 打印最终配置 (调试用)
-- ================================================================
return options
