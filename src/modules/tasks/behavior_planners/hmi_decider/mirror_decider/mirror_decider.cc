
// version 3
#include "mirror_decider.h"
#include "planning_context.h"
#include "environmental_model.h"
#include "debug_info_log.h"
#include "func_state_machine_c.h"

namespace planning {

MirrorDecider::MirrorDecider(const EgoPlanningConfigBuilder *config_builder,
                             framework::Session *session)
    : Task(config_builder, session) {
  current_mirror_state_ = iflyauto::REAR_VIEW_MIRROR_NONE;
  unfold_frame_counter_ = 0;
}

bool MirrorDecider::Execute() {

  // 设置阈值  后视镜折叠条件的阈值、后视镜展开条件的阈值，车停止时的阈值
  const double fold_threshold = 0.20;
  const double unfold_threshold = 0.40;
  const double stop_threshold = 0.10;
  // 设置4秒规划 角点的参数
  double s_front_left_traj = 1.00;
  double l_front_left_traj = 1.00;
  double s_front_right_traj = 1.00;
  double l_front_right_traj = 1.00;
  double front_left_l1_traj = 1.00;
  double front_left_l2_traj = 1.00;
  double front_right_l1_traj = 1.00;
  double front_right_l2_traj = 1.00;
  double path_heading_traj = 1.00;
  double heading_angle_traj = 1.00;
  const bool is_nsa_scene = session_->is_nsa_scene();
  const bool nsa_completed = session_->planning_context().nsa_planning_completed();
  const auto nsa_func_state = session_->environmental_model().get_local_view().function_state_machine_info.current_state;

  // 获取输出的窄路信息
  const auto& narrow_output = session_->planning_context().narrow_space_decider_output();
  if (!narrow_output.is_exist_narrow_space) {
    return true;
  }
  double narrow_start_s = narrow_output.narrow_space_s_range.first;   //narrow_space_s_range 窄路区间的起始点
  double narrow_end_s   = narrow_output.narrow_space_s_range.second;  //narrow_space_s_range 窄路区间的终止点

  // 获取真正包含完整 5 秒预测的原始轨迹点 (raw_traj_points)
  const auto& raw_traj_points = session_->planning_context().planning_result().raw_traj_points;

  // 获取真实车身物理尺寸
  const auto& vehicle_param = VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double front_edge   = vehicle_param.front_edge_to_rear_axle;  // 后轴到前保险杠（ 3.73m ）
  const double rear_edge    = vehicle_param.rear_edge_to_rear_axle;   // 后轴到后保险杠（ 1.085m ）
  const double max_width    = vehicle_param.max_width;                // 车宽（带后视镜 2.229m ）
  // 获取当前车辆在参考路径上的s位置
  const auto& ref_path = session_->planning_context().lane_change_decider_output().coarse_planning_info.reference_path;
  const auto& frenet_ego = ref_path->get_frenet_ego_state();
  //获取当前后轴的位置
  double ego_s = frenet_ego.s();
  double ego_1 = frenet_ego.l();
  //获取当前车头的坐标
  double head_s = frenet_ego.head_s();
  double head_1 = frenet_ego.head_l();
  //获取当前车头的朝向角
  double heading_angle = frenet_ego.heading_angle();
  // 计算路径在当前s位置的航向角
  const auto& frenet_coord = ref_path->get_frenet_coord();
  double path_heading_ego = frenet_coord->GetPathCurveHeading(ego_s);
  // 计算当前相对航向角（关键计算）
  double heading_angle_ego = planning_math::NormalizeAngle( frenet_ego.heading_angle() - path_heading_ego);

  double ego_velocity = frenet_ego.velocity();//当前车速
  //获取当前车辆的四个角点的frenet坐标信息
  const auto& ego_corners = frenet_ego.corners();
  // 仿照frenet_ego.corners()来写加上后视镜后车宽的角点(只需要前头2个角点)

  double s_front_left_ego =
      ego_s + vehicle_param.front_edge_to_rear_axle * std::cos(heading_angle) -
      vehicle_param.max_width / 2.0 * std::sin(heading_angle);
  double l_front_left_ego =
      ego_1 + vehicle_param.front_edge_to_rear_axle * std::sin(heading_angle) +
      vehicle_param.max_width / 2.0 * std::cos(heading_angle);
  double s_front_right_ego =
      ego_s + vehicle_param.front_edge_to_rear_axle * std::cos(heading_angle) +
      vehicle_param.max_width / 2.0 * std::sin(heading_angle);
  double l_front_right_ego =
      ego_1 + vehicle_param.front_edge_to_rear_axle * std::sin(heading_angle) -
      vehicle_param.max_width / 2.0 * std::cos(heading_angle);

  //计算 当前车辆的前方2个角点距离左右两侧障碍物的距离
  double front_left_l1_ego  = narrow_output.left_boundary_spline(s_front_left_ego)   - l_front_left_ego;   // 角点：左前 左侧障碍物距离（正）
  double front_left_l2_ego  = narrow_output.right_boundary_spline(s_front_left_ego)  - l_front_left_ego;   // 角点：左前 右侧障碍物距离（负）
  double front_right_l1_ego = narrow_output.left_boundary_spline(s_front_right_ego)  - l_front_right_ego;  // 角点：右前 左侧障碍物距离（正）
  double front_right_l2_ego = narrow_output.right_boundary_spline(s_front_right_ego) - l_front_right_ego;  // 角点：右前 右侧障碍物距离（负）

  // 定义最高优先级标志位：当前，或未来4秒内，是否需要折叠？
  bool condition_to_fold = false;

  // 只有在 NSA 功能被激活且未完成的大前提下，才需要做判定
  if (ego_velocity != 0 && is_nsa_scene && !nsa_completed && nsa_func_state != iflyauto::FunctionalState_NRA_COMPLETED) {

    // 前提条件：必须让 左前角点 或者 右前角点 位于窄路范围内
    // 条件1：适合慢速，当前4秒规划还没有到车头的位置
    if((s_front_left_ego >= narrow_start_s && s_front_left_ego <= narrow_end_s + front_edge) || (s_front_right_ego >= narrow_start_s && s_front_right_ego <= narrow_end_s + front_edge)){
      if (front_left_l1_ego <= fold_threshold || front_left_l2_ego >= -fold_threshold || front_right_l1_ego <= fold_threshold || front_right_l2_ego >= -fold_threshold) {
        condition_to_fold = true;
      }
    }
    // 条件2：4秒的规划已经超出车头部分
    if (narrow_output.is_exist_narrow_space) {
      // 确保原始轨迹数据不为空
      if (!raw_traj_points.empty()) {
        for (const auto& pt : raw_traj_points) {
          // 只预测未来4秒
          if (pt.t > 4.0) {
            break;
          }

          //计算未来4秒的相对航向角
          heading_angle_traj = planning_math::NormalizeAngle( pt.heading_angle - frenet_coord->GetPathCurveHeading(pt.s));
          // 计算4秒内的前2个角点的轨迹点
          s_front_left_traj =
              pt.s + vehicle_param.front_edge_to_rear_axle * std::cos(heading_angle_traj) -
              vehicle_param.max_width / 2.0 * std::sin(heading_angle_traj);
          l_front_left_traj =
              pt.l + vehicle_param.front_edge_to_rear_axle * std::sin(heading_angle_traj) +
              vehicle_param.max_width / 2.0 * std::cos(heading_angle_traj);
          s_front_right_traj =
              pt.s + vehicle_param.front_edge_to_rear_axle * std::cos(heading_angle_traj) +
              vehicle_param.max_width / 2.0 * std::sin(heading_angle_traj);
          l_front_right_traj =
              pt.l + vehicle_param.front_edge_to_rear_axle * std::sin(heading_angle_traj) -
              vehicle_param.max_width / 2.0 * std::cos(heading_angle_traj);

          //计算 未来4秒轨迹的车辆的前方2个角点距离左右两侧障碍物的距离
          front_left_l1_traj  = narrow_output.left_boundary_spline(s_front_left_traj)   - l_front_left_traj;   // 角点：左前 左侧障碍物距离（正）
          front_left_l2_traj  = narrow_output.right_boundary_spline(s_front_left_traj)  - l_front_left_traj;   // 角点：左前 右侧障碍物距离（负）
          front_right_l1_traj = narrow_output.left_boundary_spline(s_front_right_traj)  - l_front_right_traj;  // 角点：右前 左侧障碍物距离（正）
          front_right_l2_traj = narrow_output.right_boundary_spline(s_front_right_traj) - l_front_right_traj;  // 角点：右前 右侧障碍物距离（负）

          // 检查4秒内的轨迹点是否在窄路危险区域
          if((s_front_left_traj >= narrow_start_s && s_front_left_traj <= narrow_end_s + front_edge) || (s_front_right_traj >= narrow_start_s && s_front_right_traj <= narrow_end_s + front_edge)){
            if(front_left_l1_traj <= fold_threshold || front_left_l2_traj >= -fold_threshold || front_right_l1_traj <= fold_threshold || front_right_l2_traj >= -fold_threshold) {
              condition_to_fold = true;
            }
          }
        }
      }
    }
  }
  else if(ego_velocity == 0 ){
      if (front_left_l1_ego <= stop_threshold || front_left_l2_ego >= -stop_threshold || front_right_l1_ego <= stop_threshold || front_right_l2_ego >= -stop_threshold) {
        condition_to_fold = true;
      }
  }

  switch (current_mirror_state_) {
    case iflyauto::REAR_VIEW_MIRROR_NONE:{
      // 【状态 0：初始/宽路状态】
      if (condition_to_fold) {
        // 预测到未来4秒有窄路，提前折叠！
        current_mirror_state_ = iflyauto::REAR_VIEW_MIRROR_FOLD;
        unfold_frame_counter_ = 0;
      }
      break;
    }
    case iflyauto::REAR_VIEW_MIRROR_FOLD:{
      // 【状态 1：窄路通行中/即将进入窄路】
      // 每一帧都问自己："现在能展开了吗？"

      bool can_unfold_now = true;  // 假设可以展开

      // 检查当前时刻是否能展开
      if (!(front_left_l1_ego >= unfold_threshold && front_left_l2_ego <= -unfold_threshold && front_right_l1_ego >= unfold_threshold && front_right_l2_ego <= -unfold_threshold)) {
        can_unfold_now = false;  // 当前太窄，不能展开
      }

      // 检查未来4秒是否能展开
      if (can_unfold_now && !raw_traj_points.empty()) {
        for (const auto& pt : raw_traj_points) {
          if (pt.t > 4.0) {
            break;  // 只检查未来4秒
          }
          //计算未来4秒的相对航向角
          heading_angle_traj = planning_math::NormalizeAngle( pt.heading_angle - frenet_coord->GetPathCurveHeading(pt.s));
          // 计算4秒内的角点位置（复用你的计算逻辑）
          s_front_left_traj  = pt.s + vehicle_param.front_edge_to_rear_axle * std::cos(heading_angle_traj) - vehicle_param.max_width / 2.0 * std::sin(heading_angle_traj);
          l_front_left_traj  = pt.l + vehicle_param.front_edge_to_rear_axle * std::sin(heading_angle_traj) + vehicle_param.max_width / 2.0 * std::cos(heading_angle_traj);
          s_front_right_traj = pt.s + vehicle_param.front_edge_to_rear_axle * std::cos(heading_angle_traj) + vehicle_param.max_width / 2.0 * std::sin(heading_angle_traj);
          l_front_right_traj = pt.l + vehicle_param.front_edge_to_rear_axle * std::sin(heading_angle_traj) - vehicle_param.max_width / 2.0 * std::cos(heading_angle_traj);

          // 计算距离
          front_left_l1_traj  = narrow_output.left_boundary_spline(s_front_left_traj)   - l_front_left_traj;
          front_left_l2_traj  = narrow_output.right_boundary_spline(s_front_left_traj)  - l_front_left_traj;
          front_right_l1_traj = narrow_output.left_boundary_spline(s_front_right_traj)  - l_front_right_traj;
          front_right_l2_traj = narrow_output.right_boundary_spline(s_front_right_traj) - l_front_right_traj;

          // 只要未来4秒内有任意时刻太窄，就不能展开
          if((s_front_left_traj >= narrow_start_s && s_front_left_traj <= narrow_end_s + front_edge) || (s_front_right_traj >= narrow_start_s && s_front_right_traj <= narrow_end_s + front_edge)){
            if (!(front_left_l1_traj >= unfold_threshold && front_left_l2_traj <= -unfold_threshold && front_right_l1_traj >= unfold_threshold && front_right_l2_traj <= -unfold_threshold)) {
              can_unfold_now = false;
              break;  // 发现危险，立即跳出循环
            }
          }
        }
      }
      // 只有当前和未来4秒都安全，才允许展开
      if (can_unfold_now) {
        current_mirror_state_ = iflyauto::REAR_VIEW_MIRROR_UNFOLD;
        unfold_frame_counter_ = 0;
      }
      break;
    }
    case iflyauto::REAR_VIEW_MIRROR_UNFOLD:{
      // 【状态 2：刚出窄路，准备展开，进行防抖延迟计数】
      if (condition_to_fold) {
        // 展开到一半，预测到 4 秒后又有一段窄路，最高优先级：立刻重新折叠！
        current_mirror_state_ = iflyauto::REAR_VIEW_MIRROR_FOLD;
        unfold_frame_counter_ = 0;
      }
      else {
        // 安全处于宽路，计数器累加
        unfold_frame_counter_++;

        // 持续了 5 帧 (0.5秒) 安全态，彻底复位
        if (unfold_frame_counter_ >= 5) {
          current_mirror_state_ = iflyauto::REAR_VIEW_MIRROR_NONE;
          unfold_frame_counter_ = 0;
        }
      }
      break;
    }
    default:
      current_mirror_state_ = iflyauto::REAR_VIEW_MIRROR_NONE;
      unfold_frame_counter_ = 0;
      break;
  }

  auto& rear_mirror_cmd = session_->mutable_planning_context()->mutable_planning_output().rear_view_mirror_signal_command;
  if (current_mirror_state_ == iflyauto::REAR_VIEW_MIRROR_NONE) {
    rear_mirror_cmd.available = false;
    rear_mirror_cmd.rear_view_mirror_value = iflyauto::REAR_VIEW_MIRROR_NONE;
  } else {
    rear_mirror_cmd.available = true;
    rear_mirror_cmd.rear_view_mirror_value = current_mirror_state_;
  }

  return true;
}

}  // namespace planning

