-- 引入Cartographer核心建图器基础配置（ROS2 Humble）
include "map_builder.lua"
-- 引入轨迹构建器基础配置（ROS2 Humble）
include "trajectory_builder.lua"

-- 全局配置选项
options = {
  map_builder = MAP_BUILDER,                  -- 关联建图器配置实例
  trajectory_builder = TRAJECTORY_BUILDER,    -- 关联轨迹构建器配置实例
  map_frame = "map",                          -- 地图坐标系（ROS标准，固定为map）
  tracking_frame = "imu_link",                -- 跟踪坐标系（imu）
  published_frame = "odom",                   -- 发布的里程计坐标系（ROS标准）
  odom_frame = "odom",                        -- 里程计坐标系（由麦轮小车里程计提供）
  provide_odom_frame = false,                 -- 不由Cartographer生成odom（使用小车自身odom）
  publish_frame_projected_to_2d = false,      -- 不投影坐标系到2D，保留3D信息适配IMU
  use_odometry = true,                        -- 启用里程计（麦轮小车必须开启）
  use_nav_sat = false,                        -- 不使用GPS（无卫星模块）
  use_landmarks = false,                      -- 不使用地标（无人工地标）
  num_laser_scans = 1,                        -- 激光雷达数量（1个单线雷达）
  num_multi_echo_laser_scans = 0,             -- 无多回波激光雷达
  num_subdivisions_per_laser_scan = 1,        -- 激光扫描细分次数（默认1，保证实时性）
  num_point_clouds = 0,                       -- 无点云输入（仅用激光）
  lookup_transform_timeout_sec = 0.5,         -- TF变换查找超时（适配树莓派算力延迟）
  submap_publish_period_sec = 0.5,            -- 子图发布周期（降低树莓派开销）
  pose_publish_period_sec = 10e-3,            -- 位姿发布周期（提升稳定性）
  trajectory_publish_period_sec = 50e-3,      -- 轨迹发布周期（适配树莓派）
  rangefinder_sampling_ratio = 1.,            -- 激光采样率100%（保证建图精度）
  odometry_sampling_ratio = 1.,               -- 里程计采样率100%（麦轮小车需完整里程计）
  fixed_frame_pose_sampling_ratio = 1.,       -- 固定坐标系位姿采样率100%
  imu_sampling_ratio = 1.,                    -- IMU采样率100%（滤波后IMU数据稳定）
  landmarks_sampling_ratio = 1.,              -- 地标采样率100%（无地标不影响）
}

-- 启用2D轨迹构建器（2D建图）
MAP_BUILDER.use_trajectory_builder_2d = true

-- 位姿图优化配置（适配树莓派算力）
POSE_GRAPH.optimize_every_n_nodes = 50        -- 每50个节点优化一次
POSE_GRAPH.constraint_builder.min_score = 0.65-- 约束匹配最低分数
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65 -- 全局定位匹配分数（同步降低）

-- 2D轨迹构建器核心配置（适配麦轮+IMU）
TRAJECTORY_BUILDER_2D.min_range = 0.1         -- 激光最小检测距离（过滤0.1m内噪声）
TRAJECTORY_BUILDER_2D.max_range = 8.0         -- 激光最大检测距离（8m，匹配单线雷达性能）
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 0.5 -- 缺失数据的射线长度（默认0.5m）
TRAJECTORY_BUILDER_2D.use_imu_data = true     -- 启用IMU数据（你的IMU已滤波，提升建图稳定性）
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0 -- IMU重力时间常数

-- 运动滤波配置（核心适配麦轮全向运动）
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2) -- 角度阈值
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05        -- 平移阈值
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5            -- 时间阈值

-- 启用在线相关扫描匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 返回配置（Cartographer加载配置必需）
return options