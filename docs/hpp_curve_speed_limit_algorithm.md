# HPP 弯道限速算法文档

## 1. 算法概述

HPP（Highway Pilot Parking）场景下的弯道限速算法是基于车辆运动学和动力学约束，通过计算路径曲率与横向加速度的关系，确保车辆在弯道行驶时的舒适性和安全性。该算法主要包含两个层级的限速策略：

1. **基于横向加速度的物理限速**：根据路径曲率和最大横向加速度计算理论限速
2. **HPP 场景特定限速**：在高曲率路段应用更严格的限速策略

## 2. 算法原理

### 2.1 核心物理公式

车辆在弯道行驶时，向心加速度与速度和曲率的关系为：

```
a_lateral = v² × κ
```

其中：
- `a_lateral`：横向加速度 (m/s²)
- `v`：车辆速度 (m/s)
- `κ`：路径曲率 (1/m)

由此推导出基于横向加速度的速度限制：

```
v_limit = sqrt(a_max_lateral / κ)
```

### 2.2 曲率计算

算法综合考虑两种曲率：

1. **路径曲率（Path Curvature）**：从横向规划轨迹点中提取的最大曲率
2. **方向盘曲率（Steering Curvature）**：基于当前方向盘转角计算

```
κ_steering = tan(δ / steer_ratio) / wheel_base
```

最终使用的曲率为两者的最大值：

```
κ_max = max(κ_path, |κ_steering|)
```

## 3. 关键参数

### 3.1 物理参数

| 参数名称 | 数值 | 单位 | 说明 |
|---------|------|------|------|
| `urban_max_lat_acceleration` | 2.0 | m/s² | 城市道路最大横向加速度 |
| `highway_max_lat_acceleration` | 2.5 | m/s² | 高速道路最大横向加速度 |
| `velocity_lower_bound` | 1.39 | m/s | 速度下限（5 km/h） |
| `velocity_upper_bound` | 34.72 | m/s | 速度上限（125 km/h） |

### 3.2 HPP 场景参数

| 参数名称 | 数值 | 单位 | 说明 |
|---------|------|------|------|
| `velocity_limit_parking` | 10.0 | km/h | 停车场景速度限制 |
| `kVelocityPreviewDistance` | ego_v × 1.0 | m | 速度预瞄距离（随当前车速变化） |
| `kappa_threshold` | 0.07 | 1/m | 触发严格限速的曲率阈值 |
| `min_curve_speed` | 2.0 | m/s | 弯道最小限速 |
| `max_curve_speed` | 5.0 | km/h | 高曲率弯道最大限速 |

### 3.3 横向加速度动态调整参数

| 场景 | 速度阈值 | 加速度缓冲最大值 | 加速度增长率 |
|------|---------|----------------|-------------|
| 城市道路 | 3.0 m/s | 0.7 m/s² | 0.2 |
| 高速道路 | 20.0 m/s | 0.5 m/s² | 0.05 |

## 4. 算法流程

### 4.1 总体流程图

```
┌─────────────────────────────────┐
│  获取轨迹点和车辆状态           │
└────────────┬────────────────────┘
             │
             ▼
┌─────────────────────────────────┐
│  计算路径最大曲率               │
│  - 遍历轨迹点                   │
│  - 记录最大曲率位置             │
└────────────┬────────────────────┘
             │
             ▼
┌─────────────────────────────────┐
│  计算方向盘曲率                 │
│  κ_steer = tan(δ/ratio)/L       │
└────────────┬────────────────────┘
             │
             ▼
┌─────────────────────────────────┐
│  确定最大曲率                   │
│  κ_max = max(κ_path, κ_steer)   │
└────────────┬────────────────────┘
             │
             ▼
┌─────────────────────────────────┐
│  计算最大横向加速度             │
│  - 根据道路类型和速度           │
│  - 考虑变道状态                 │
└────────────┬────────────────────┘
             │
             ▼
┌─────────────────────────────────┐
│  计算基于曲率的速度限制         │
│  v_limit = sqrt(a_lat_max / κ)  │
└────────────┬────────────────────┘
             │
             ▼
┌─────────────────────────────────┐
│  是否为 HPP 场景？              │
└────┬────────────────────┬───────┘
    否│                   │是
     │                    ▼
     │         ┌─────────────────────────┐
     │         │  获取预瞄点曲率          │
     │         │  s_preview = s + v×1.0  │
     │         └─────────┬───────────────┘
     │                   │
     │                   ▼
     │         ┌─────────────────────────┐
     │         │  |κ| > 0.07 ?           │
     │         └─────┬──────────┬────────┘
     │              否│         │是
     │               │          ▼
     │               │  ┌──────────────────────┐
     │               │  │  应用严格限速         │
     │               │  │  v = (1-|κ|) × v_map │
     │               │  │  v = min(v, 5km/h)   │
     │               │  │  v = max(v, 2.0m/s)  │
     │               │  └──────────┬───────────┘
     │               │             │
     └───────────────┴─────────────┘
                     │
                     ▼
          ┌─────────────────────┐
          │  综合各类限速        │
          │  - 曲率限速          │
          │  - 地图限速          │
          │  - 用户限速          │
          │  - 窄区域限速        │
          └─────────┬───────────┘
                    │
                    ▼
          ┌─────────────────────┐
          │  输出最终速度限制    │
          └─────────────────────┘
```

### 4.2 详细步骤

#### 步骤 1：计算路径最大曲率

```cpp
double max_curv_s = -1.0;
double path_curvature = 1e-4;
double s_accumulate = 0.0;
for (size_t i = 0; i < traj_points.size(); ++i) {
  if (std::fabs(traj_points[i].curvature) > path_curvature) {
    path_curvature = std::fabs(traj_points[i].curvature);
    max_curv_s = s_accumulate;  // 记录最大曲率位置
  }
  if (i > 0) {
    s_accumulate += hypot(traj_points[i].x - traj_points[i-1].x,
                          traj_points[i].y - traj_points[i-1].y);
  }
}
```

#### 步骤 2：计算方向盘曲率

```cpp
double steer_angle = ego_state->ego_steer_angle();
double steer_curvature = tan(steer_angle / steer_ratio) / wheel_base;
double max_curvature = max(path_curvature, fabs(steer_curvature));
```

#### 步骤 3：计算最大横向加速度

最大横向加速度根据道路类型和车速动态调整：

**高速道路**：
```
a_lat_max = highway_max_lat_acceleration + 
            min(0.5, 0.05 × max(v_ego - 20.0, 0.0))
```

**城市道路**：
```
a_lat_max = urban_max_lat_acceleration + 
            min(0.7, 0.2 × max(v_ego - 3.0, 0.0)) - 
            lc_acc_decay
```

其中：
- `lc_acc_decay`：低速变道时的加速度衰减，在低速（< 3 m/s）左右变道时为 1.5 m/s²

#### 步骤 4：计算基于曲率的速度限制

```cpp
double v_limit_curv = 
    max(velocity_lower_bound,
        min(velocity_upper_bound,
            sqrt(max_lat_acceleration / max_curvature)));
```

#### 步骤 5：HPP 场景特殊处理

在 HPP 场景下，对高曲率路段应用更严格的限速：

```cpp
if (session_->is_hpp_scene()) {
  // 预瞄距离随速度变化
  double kVelocityPreviewDistance = ego_v × 1.0;
  
  // 获取预瞄点
  ReferencePathPoint refpath_pt;
  reference_path_ptr->get_reference_point_by_lon(
      frenet_ego_state.s() + kVelocityPreviewDistance, refpath_pt);
  
  // 高曲率判断
  if (fabs(refpath_pt.path_point.kappa()) > 0.07) {
    // 应用严格限速公式
    user_velocity_limit = 
        min((1.0 - fabs(refpath_pt.path_point.kappa())) × map_velocity_limit,
            5.0 / 3.6);  // 最高 5 km/h
    user_velocity_limit = max(user_velocity_limit, 2.0);  // 最低 2 m/s
  }
}
```

**限速公式解析**：
- `(1.0 - |κ|) × map_velocity_limit`：曲率越大，限速越低
  - 当 κ = 0.07 时，限速为 93% 的地图限速
  - 当 κ = 0.5 时，限速为 50% 的地图限速
- 上限：不超过 5 km/h（≈1.39 m/s）
- 下限：不低于 2.0 m/s

#### 步骤 6：综合限速

最终速度限制取各类限速的最小值：

```cpp
final_velocity_limit = min({
    v_limit_curv,           // 曲率限速
    map_velocity_limit,     // 地图限速
    user_velocity_limit,    // 用户/HPP限速
    narrow_area_velocity    // 窄区域限速
});
```

### 4.3 brake_traj 限速轨迹应用流程

在计算出 `final_velocity_limit` 后，算法会创建一条舒适减速参考轨迹（`brake_traj`），并使用该轨迹约束纵向规划的速度上界。以下是详细的应用流程：

```
┌─────────────────────────────────────┐
│  计算弯道限速 final_velocity_limit   │
│  (包含所有曲率限速约束)              │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  创建 brake_traj 舒适减速轨迹       │
│  - 初始位置: s0                     │
│  - 初始速度: v0 (当前速度)          │
│  - 初始加速度: a0                   │
│  - Jerk: -1.0 m/s³ (舒适制动)       │
│  - 速度下界: final_velocity_limit   │
│  - 速度上界: ∞                      │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  set_bound() 设置轨迹边界           │
│  v_min = final_velocity_limit       │
│  (防止过度减速)                     │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  返回轨迹对象                       │
│  map_velocity_limit_brake_traj      │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  遍历每个规划时刻 t_i               │
│  (通常 dt=0.2s, 总时长 5s)          │
│  共 26 个时刻点                     │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  调用 evaluate(0, t_i) 获取 s_i    │
│  计算在 t_i 时刻的参考位置          │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  更新位置约束                       │
│  s_ref = min(s_ref, s_i)            │
│  (确保不超出限速轨迹)               │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  调用 evaluate(1, t_i) 获取 v_i    │
│  计算在 t_i 时刻的参考速度          │
│  (v_i 受 final_velocity_limit 约束) │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  更新速度约束上界                   │
│  bound_v.upper = min(bound_v.upper, │
│                      v_i)           │
│  (实现曲率限速约束)                 │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  更新 lon_ref_path 约束边界         │
│  - lon_ref_path.s_refs[i]           │
│  - lon_ref_path.lon_bound_v[i]      │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  约束传递给纵向运动规划器           │
│  LongitudinalMotionPlanner          │
│  通过 planning_context 共享         │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  iLQR 优化求解                      │
│  在速度约束下生成最优轨迹           │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  输出满足曲率限速的纵向轨迹         │
└─────────────────────────────────────┘
```

**关键要点**：

1. **v_min 的作用**：`brake_traj` 的 `v_min = final_velocity_limit` 确保参考轨迹不会减速到低于目标限速，避免过度减速。

2. **约束传递机制**：
   - `brake_traj` 模拟从当前速度舒适减速到限速的过程
   - 每个时刻 t_i 的速度 v_i 作为该时刻的速度上界
   - 通过 `bound_v.upper = min(bound_v.upper, v_i)` 实现约束

3. **物理意义**：
   - 当前速度 > 限速：生成减速轨迹，逐步降低速度上界
   - 当前速度 ≤ 限速：轨迹保持在限速值，速度上界固定
   - 保证舒适性：使用恒定 jerk = -1.0 m/s³ 实现平滑减速

4. **时间参数**：
   - 采样间隔 `delta_time`：通常为 0.2 秒
   - 规划时长：5 秒（可配置）
   - 总采样点：26 个（0, 0.2, 0.4, ..., 5.0s）

## 5. 代码实现

### 5.1 主函数

文件：`general_longitudinal_decider.cc`

函数：`GeneralLongitudinalDecider::get_velocity_limit()`

关键代码段：

```605:754:src/modules/tasks/behavior_planners/general_longitudinal_decider/general_longitudinal_decider.cc
BoundedConstantJerkTrajectory1d GeneralLongitudinalDecider::get_velocity_limit(
    const LongitudinalDeciderOutput &lon_ref_path) {
  // NTRACE_CALL(9);

  // get curvature velocity limit
  const auto ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto &map_info_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &traj_points = session_->mutable_planning_context()
                                ->mutable_planning_result()
                                .traj_points;
  // 查找横向优化后路径上最大曲率，以及最大曲率处距离当前位置的距离
  double max_curv_s = -1.0;
  double path_curvature = 1e-4;
  double s_accumulate = 0.0;
  for (size_t i = 0; i < traj_points.size(); ++i) {
    if (std::fabs(traj_points[i].curvature) > path_curvature) {
      path_curvature = std::fabs(traj_points[i].curvature);
      max_curv_s = s_accumulate;
    }
    if (i > 0) {
      s_accumulate +=
          planning::fast_hypot(traj_points[i].x - traj_points[i - 1].x,
                               traj_points[i].y - traj_points[i - 1].y);
    }
  }
  LOG_DEBUG(
      "[VirtualLaneManager::get_velocity_limit] path_curvature: %f, "
      "max_curv_s: %f \n",
      path_curvature, max_curv_s);
  double steer_angle = ego_state->ego_steer_angle();
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double steer_curvature = std::tan(steer_angle / vehicle_param.steer_ratio) /
                           vehicle_param.wheel_base;
  double max_curvature = std::max(path_curvature, std::fabs(steer_curvature));
  const double max_lat_acceleration = compute_max_lat_acceleration();
  double v_limit_path_curv =
      std::max(config_.velocity_lower_bound,
               std::min(config_.velocity_upper_bound,
                        std::sqrt(max_lat_acceleration / path_curvature)));
  double v_limit_curv =
      std::max(config_.velocity_lower_bound,
               std::min(config_.velocity_upper_bound,
                        std::sqrt(max_lat_acceleration / max_curvature)));
  vel_limit_info_.v_limit_curv = v_limit_curv;
  max_curvature_ = max_curvature;
  double vlimit_jerk = 0.0;
  double vlimit_acc = 0.0;
  double time_to_brake = 1e-2;
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  auto planning_init_point =
      reference_path_ptr->get_frenet_ego_state().planning_init_point();
  const double ego_v = planning_init_point.v;
  const double ego_a =
      std::abs(planning_init_point.a) < 1e-2 ? 1e-2 : planning_init_point.a;
  const double ego_jerk = planning_init_point.jerk;
  const double b_square_minus_4ac =
      std::pow((2 * ego_v + v_limit_path_curv), 2) + 6 * ego_a * max_curv_s;
  // 曲率限速要低于当前速度 && 最大曲率距离>0 && 根号内值大于0
  const double kTimeDistanceThreshold = 0.5;
  const bool enable_vlimit_curv_dec =
      v_limit_path_curv < ego_v &&
      max_curv_s > ego_v * kTimeDistanceThreshold && b_square_minus_4ac > 0.0;
  if (enable_vlimit_curv_dec) {
    time_to_brake =
        (-(2 * ego_v + v_limit_path_curv) + std::sqrt(b_square_minus_4ac)) /
        ego_a;
    time_to_brake = std::max(time_to_brake, 0.01);
    vlimit_jerk = 2 * (v_limit_path_curv - ego_v - ego_a * time_to_brake) /
                  std::pow(time_to_brake, 2);
  }

  // get map velocity limit
  constexpr double kMinMapVelocityLimit = 60.0 / 3.6;
  // vel_limit_info_.v_limit_map = map_info_manager->map_velocity_limit();
  vel_limit_info_.v_limit_map =
      map_info_manager->get_current_lane()->velocity_limit();
  double map_velocity_limit =
      std::max(vel_limit_info_.v_limit_map, kMinMapVelocityLimit);
  double real_map_limit =
      vel_limit_info_.v_limit_map * config_.velocity_upper_bound_scale_rate;
  map_velocity_limit =
      map_velocity_limit * config_.velocity_upper_bound_scale_rate;
  // double user_velocity_limit = map_info_manager->user_velocity_limit();
  double user_velocity_limit = ego_state->ego_v_cruise();
  vel_limit_info_.v_limit_usr = user_velocity_limit;
  bool disable_user_limit =
      (std::fabs(user_velocity_limit - real_map_limit) < 0.01 &&
       (user_velocity_limit < kMinMapVelocityLimit));

  // get final_velocity_limit
  double final_velocity_limit = vel_limit_info_.v_limit_curv;
  if (disable_user_limit) {
    final_velocity_limit = std::min(final_velocity_limit, kMinMapVelocityLimit);
  } else {
    final_velocity_limit =
        std::min(final_velocity_limit,
                 std::min(user_velocity_limit, map_velocity_limit));
  }

  if (session_->is_hpp_scene()) {
    // static constexpr double kVelocityPreviewDistance = 3.0;
    double kVelocityPreviewDistance = ego_v * 1.0;
    // auto mff_cruise_velocity = session_
    //                                ->environmental_model()
    //                                .get_vehicle_status()
    //                                .velocity()
    //                                .cruise_velocity()
    //                                .value_mps();
    auto mff_cruise_velocity = user_velocity_limit;
    map_velocity_limit =
        std::min(config_.velocity_limit_parking / 3.6, mff_cruise_velocity);

    // enable narrow area velocity limit in low max_curvature
    double narrow_area_velocity = std::numeric_limits<double>::max();
    narrow_area_velocity = get_narrow_area_velocity_limit();
    vel_limit_info_.v_limit_narrow_area = narrow_area_velocity;
    LOG_DEBUG("The narrow_area_velocity is = : %f", narrow_area_velocity);
    // add curvature velocity limit
    const auto &frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
    ReferencePathPoint refpath_pt;
    if (reference_path_ptr->get_reference_point_by_lon(
            frenet_ego_state.s() + kVelocityPreviewDistance, refpath_pt)) {
      user_velocity_limit = map_velocity_limit;
      if (std::fabs(refpath_pt.path_point.kappa()) > 0.07) {
        user_velocity_limit =
            std::min((1.0 - std::fabs(refpath_pt.path_point.kappa())) *
                         map_velocity_limit,
                     5 / 3.6);
        user_velocity_limit = std::max(user_velocity_limit, 2.0);
      }
      vel_limit_info_.v_limit_usr = user_velocity_limit;
      final_velocity_limit = std::min(map_velocity_limit, user_velocity_limit);
    }
    final_velocity_limit = std::min(final_velocity_limit, narrow_area_velocity);
    LOG_DEBUG("curvature: %f, curvature_velocity_limit:%f",
              refpath_pt.path_point.kappa(), user_velocity_limit);
  }

  vel_limit_info_.v_limit_final = final_velocity_limit;
```

### 5.2 横向加速度计算

```796:837:src/modules/tasks/behavior_planners/general_longitudinal_decider/general_longitudinal_decider.cc
const double GeneralLongitudinalDecider::compute_max_lat_acceleration() const {
  constexpr double kUrbanVelocityThld = 3.0;
  constexpr double kHighwayVelocityThld = 20.0;
  constexpr double kUrbanLatAccBuffMax = 0.7;
  constexpr double kHighwayLatAccBuffMax = 0.5;
  constexpr double kUrbanLatAccRate = 0.2;
  constexpr double kHighwayLatAccRate = 0.05;
  constexpr double kLowSpeedLaneChangeAccBuff = 1.5;
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const auto &frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
  auto ego_velocity = frenet_ego_state.planning_init_point().v;
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto state = coarse_planning_info.target_state;
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  bool is_LC_LCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == LEFT_CHANGE);
  bool is_LC_RCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == RIGHT_CHANGE);
  auto lc_acc_decay =
      ((is_LC_LCHANGE || is_LC_RCHANGE) && ego_velocity < kUrbanVelocityThld)
          ? kLowSpeedLaneChangeAccBuff
          : 0.0;
  if (session_->environmental_model().is_on_highway()) {
    return config_.highway_max_lat_acceleration +
           std::min(kHighwayLatAccBuffMax,
                    kHighwayLatAccRate *
                        std::max(ego_velocity - kHighwayVelocityThld, 0.0));
  } else {
    return config_.urban_max_lat_acceleration - lc_acc_decay +
           std::min(kUrbanLatAccBuffMax,
                    kUrbanLatAccRate *
                        std::max(ego_velocity - kUrbanVelocityThld, 0.0));
  }
}
```

## 6. 配置参数

### 6.1 配置文件位置

- HPP 配置：`res/conf/module_configs/general_planner_module_hpp.json`
- 代码配置：`src/modules/context/ego_planning_config.h`

### 6.2 主要配置项

**HPP 配置文件**（`general_planner_module_hpp.json`）：

```json
{
  "velocity_upper_bound": 34.72,
  "velocity_upper_bound_scale_rate": 1.2,
  "velocity_limit_parking": 10.0
}
```

**代码默认配置**（`ego_planning_config.h`）：

```cpp
double velocity_lower_bound = 1.3888889;  // 5 km/h
double urban_max_lat_acceleration = 2.0;   // m/s²
double highway_max_lat_acceleration = 2.5; // m/s²
```

## 7. 算法特点

### 7.1 优势

1. **物理约束保证**：基于横向加速度的限速确保车辆动力学安全
2. **动态调整**：横向加速度随车速和道路类型动态调整
3. **预见性**：通过预瞄距离提前感知弯道
4. **分层限速**：通用限速 + HPP 特殊限速的分层策略
5. **鲁棒性**：同时考虑路径曲率和方向盘曲率

### 7.2 适用场景

- ✅ 停车场低速行驶
- ✅ 急弯道场景
- ✅ 变道过程
- ✅ 窄通道行驶
- ✅ 高曲率路径

### 7.3 注意事项

1. **曲率阈值敏感**：0.07 的曲率阈值对应约 14.3m 的转弯半径
2. **速度依赖**：预瞄距离随速度变化，速度越快预瞄越远
3. **保守策略**：HPP 场景下限速较为保守，最高 5 km/h
4. **最小速度保证**：即使在极高曲率下，仍保证不低于 2.0 m/s

## 8. 曲率与限速对应关系

### 8.1 基于物理约束的限速（a_lat = 2.0 m/s²）

| 曲率 κ (1/m) | 转弯半径 R (m) | 限速 v (km/h) | 限速 v (m/s) |
|-------------|---------------|--------------|-------------|
| 0.01 | 100 | 50.9 | 14.1 |
| 0.02 | 50 | 36.0 | 10.0 |
| 0.05 | 20 | 22.8 | 6.3 |
| 0.07 | 14.3 | 19.2 | 5.3 |
| 0.10 | 10 | 16.1 | 4.5 |
| 0.20 | 5 | 11.4 | 3.2 |

### 8.2 HPP 高曲率限速（κ > 0.07）

假设 map_velocity_limit = 10 km/h (2.78 m/s)

| 曲率 κ (1/m) | 公式限速 (m/s) | 上限限制 (m/s) | 最终限速 (m/s) |
|-------------|---------------|---------------|---------------|
| 0.07 | 2.58 | 1.39 | **2.0** (下限) |
| 0.10 | 2.50 | 1.39 | **2.0** (下限) |
| 0.20 | 2.22 | 1.39 | **2.0** (下限) |
| 0.30 | 1.94 | 1.39 | **2.0** (下限) |
| 0.50 | 1.39 | 1.39 | **2.0** (下限) |

**结论**：在 HPP 场景下，当曲率 > 0.07 时，实际限速基本为最小值 2.0 m/s（7.2 km/h）。

## 9. 调试信息

算法提供以下调试日志：

```cpp
LOG_DEBUG("[VirtualLaneManager::get_velocity_limit] path_curvature: %f, max_curv_s: %f", 
          path_curvature, max_curv_s);
LOG_DEBUG("curvature: %f, curvature_velocity_limit:%f", 
          refpath_pt.path_point.kappa(), user_velocity_limit);
```

速度限制信息结构体：

```cpp
struct VelocityLimitInfo {
  double v_limit_map;           // 地图限速
  double v_limit_usr;           // 用户/HPP限速
  double v_limit_curv;          // 曲率限速
  double v_limit_narrow_area;   // 窄区域限速
  double v_limit_final;         // 最终限速
};
```

## 10. 版本信息

- **文档版本**：v1.0
- **创建日期**：2026-01-30
- **代码文件**：`general_longitudinal_decider.cc`
- **代码行数**：605-754（主函数），796-837（横向加速度计算）
- **分支**：`zdjiang3/add_hpp_stop_decider`

---

## 附录

### A. 相关函数接口

```cpp
// 主函数
BoundedConstantJerkTrajectory1d get_velocity_limit(
    const LongitudinalDeciderOutput &lon_ref_path);

// 横向加速度计算
const double compute_max_lat_acceleration() const;

// 窄区域限速
double get_narrow_area_velocity_limit();
```

### B. 参考资料

1. 车辆动力学基础理论
2. 横向加速度与乘坐舒适性关系
3. RSS (Responsibility-Sensitive Safety) 模型

### C. 后续优化建议

1. 曲率阈值可配置化
2. 根据路面附着系数动态调整横向加速度
3. 考虑载重对横向稳定性的影响
4. 增加驾驶风格参数（舒适/运动模式）

### D. 模块间数据传递流程

本节描述弯道限速约束如何从行为决策模块传递到运动规划模块。

#### D.1 数据流架构

```
┌─────────────────────────────────────────────────────────────┐
│                    HPP Task Pipeline                        │
│                  (hpp_task_pipeline.cc)                     │
│                                                              │
│  任务执行顺序：                                              │
│  1. LaneChangeDecider (参考路径生成)                        │
│  2. LateralObstacleDecider (横向障碍物决策)                 │
│  3. HppGeneralLateralDecider (横向行为决策)                 │
│  4. LateralMotionPlanner (横向轨迹规划)                     │
│  5. HppStopDecider (停车决策)                               │
│  6. GeneralLongitudinalDecider (纵向行为决策) ← 计算限速    │
│  7. LongitudinalMotionPlanner (纵向运动规划) ← 应用限速     │
│  8. ResultTrajectoryGenerator (轨迹生成)                    │
└─────────────────────────────────────────────────────────────┘
```

#### D.2 数据共享机制

**通过 `session_->planning_context()` 共享数据**：

```
┌────────────────────────────────┐
│  GeneralLongitudinalDecider    │
│  (纵向行为决策器)              │
└────────────┬───────────────────┘
             │
             │ 写入数据 (mutable)
             │
             ▼
┌─────────────────────────────────────────────┐
│    session_->planning_context()             │
│                                              │
│  ┌────────────────────────────────────────┐ │
│  │  longitudinal_decider_output           │ │
│  │                                        │ │
│  │  - s_refs: 位置参考                   │ │
│  │  - ds_refs: 速度参考                  │ │
│  │  - hard_bounds: 硬约束边界            │ │
│  │  - lon_bound_v: 速度约束 ← 曲率限速   │ │
│  │  - lon_bound_a: 加速度约束            │ │
│  │  - lon_bound_jerk: Jerk约束           │ │
│  │  - lon_lead_bounds: 跟车约束          │ │
│  └────────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
             │
             │ 读取数据 (const)
             │
             ▼
┌────────────────────────────────┐
│  LongitudinalMotionPlanner     │
│  (纵向运动规划器)              │
└────────────────────────────────┘
```

#### D.3 关键代码调用链

**1. GeneralLongitudinalDecider 写入数据**：

```cpp
// general_longitudinal_decider.cc: 106-108
auto &lon_ref_path = session_->mutable_planning_context()
                         ->mutable_longitudinal_decider_output();
lon_ref_path.Clear();

// 计算曲率限速并填充 lon_bound_v
// 第 380 行
set_velocity_acceleration_bound(lon_ref_path);

// 第 869-871 行：在 set_velocity_acceleration_bound 中
bound_v.upper = std::min(bound_v.upper, kVelocityUpperBound);
auto v_i = map_velocity_limit_brake_traj.evaluate(1, relative_time);
bound_v.upper = std::min(bound_v.upper, v_i);  // ← 包含曲率限速
```

**2. LongitudinalMotionPlanner 读取数据**：

```cpp
// longitudinal_motion_planner.cc: 81-90
void LongitudinalMotionPlanner::AssembleInput() {
  const auto &longitudinal_decider_output =
      session_->planning_context().longitudinal_decider_output();
  
  const auto &v_bounds = longitudinal_decider_output.lon_bound_v;
  
  // 第 177-181 行：传递给优化器
  for (size_t i = 0; i < v_bounds.size(); ++i) {
    planning_input_.mutable_vel_max_vec()->Set(i, v_bounds[i].upper);
    planning_input_.mutable_vel_min_vec()->Set(i, v_bounds[i].lower);
  }
}
```

**3. iLQR 优化求解**：

```cpp
// 第 229 行
planning_problem_ptr_->Update(planning_input_);  // ← 在约束下优化
```

#### D.4 完整数据传递时序图

```
时刻 T0: HPP Task Pipeline 开始执行
  │
  ├─ T1: LaneChangeDecider 生成参考路径
  │       └─> planning_context.coarse_planning_info.reference_path
  │
  ├─ T2: LateralMotionPlanner 生成横向轨迹
  │       └─> planning_context.planning_result.traj_points (含曲率)
  │
  ├─ T3: GeneralLongitudinalDecider 执行
  │       │
  │       ├─ 读取横向轨迹点和曲率
  │       ├─ 计算 path_curvature (最大曲率)
  │       ├─ 计算 v_limit_curv = sqrt(a_lat_max / κ)
  │       ├─ HPP 场景：应用高曲率限速 (κ > 0.07)
  │       ├─ 综合得到 final_velocity_limit
  │       ├─ 创建 brake_traj (v_min = final_velocity_limit)
  │       ├─ 遍历规划时刻，更新 lon_bound_v[i].upper
  │       │   └─> v_upper = min(v_upper, brake_traj.evaluate(1, t_i))
  │       │
  │       └─> 写入 planning_context.longitudinal_decider_output
  │
  ├─ T4: LongitudinalMotionPlanner 执行
  │       │
  │       ├─ 读取 longitudinal_decider_output
  │       ├─ AssembleInput(): v_bounds → vel_max_vec
  │       ├─ Update(): 调用 iLQR 优化
  │       │   └─> 在速度约束 v ≤ v_max 下求解最优轨迹
  │       │
  │       └─> 写入 planning_context.motion_planner_output
  │
  └─ T5: ResultTrajectoryGenerator 生成最终轨迹
          └─> 满足曲率限速的完整轨迹输出 ✅
```

#### D.5 数据结构定义

**LongitudinalDeciderOutput** (主要字段)：

```cpp
struct LongitudinalDeciderOutput {
  std::vector<std::pair<double, double>> s_refs;     // 位置参考 [(s, weight)]
  std::vector<std::pair<double, double>> ds_refs;    // 速度参考 [(v, weight)]
  std::vector<std::vector<WeightedBound>> hard_bounds;  // 位置硬约束
  std::vector<Bound> lon_bound_v;     // 速度约束 [v_min, v_max]
  std::vector<Bound> lon_bound_a;     // 加速度约束 [a_min, a_max]
  std::vector<Bound> lon_bound_jerk;  // Jerk约束 [j_min, j_max]
  std::vector<WeightedLonLeadBounds> lon_lead_bounds;  // 跟车约束
  std::vector<double> t_list;         // 时间列表
};
```

**Bound** (约束边界)：

```cpp
struct Bound {
  double lower;  // 下界
  double upper;  // 上界 ← 曲率限速体现在这里
};
```

#### D.6 关键设计模式

1. **发布-订阅模式**：通过 `planning_context` 实现模块间解耦
2. **引用传递**：避免数据拷贝，提高效率
3. **分层约束**：
   - 行为层 (Decider)：计算约束边界
   - 运动层 (Planner)：在约束下优化
4. **时序保证**：Pipeline 确保模块按顺序执行

#### D.7 调试方法

**查看中间数据**：

```cpp
// 在 GeneralLongitudinalDecider 中
LOG_DEBUG("curvature: %f, curvature_velocity_limit:%f",
          refpath_pt.path_point.kappa(), user_velocity_limit);

// 在 LongitudinalMotionPlanner 中
// 可以打印 v_bounds[i].upper 查看速度约束
```

**Protobuf 调试信息**：

```cpp
// general_longitudinal_decider.cc: 596
GenerateLonRefPathPB(lon_ref_path);  // 生成调试 PB 数据
```

**JSON 调试输出**：

```cpp
JSON_DEBUG_VALUE("LonBehavior_v_limit_curv", vel_limit_info_.v_limit_curv);
JSON_DEBUG_VALUE("LonBehavior_v_limit_final", vel_limit_info_.v_limit_final);
```
