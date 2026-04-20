import bokeh.plotting as bkp
from bokeh.models import WheelZoomTool, HoverTool
from bokeh.plotting import ColumnDataSource

def load_time_cost_fig(bag_loader):
  plan_debug_Astar = ColumnDataSource(data ={'t_plan_debug': [],'st_graph_searcher_cost': []})

  SccLonBehaviorCostTime_vec = []
  SccLonMotionCostTime_vec = []
  SccLateralMotionCostTime_vec = []
  SccLateralBehaviorCostTime_vec = []
  EnvironmentalModelManagerCost_vec = []
  GeneralPlannerModuleCostTime_vec = []
  DynamicWorldAverageCostTime_vec = []
  st_graph_searcher_cost_vec = []
  ARAStarTime_vec = []
  LateralMotionPlannerTime_vec = []
  GeneralLateralDeciderCostTime_vec = []
  LateralObstacleDeciderTime_vec = []
  LaneChangeDeciderTime_vec = []

  # 新增字段（按你提供的列表创建
  TaskFunctionCost_vec = []
  ego_state_update_cost_vec = []
  update_route_info_cost_vec = []
  virtual_lane_manager_cost_vec = []
  traffic_light_decision_cost_vec = []
  obstacle_prediction_cost_vec = []
  obstacle_manager_cost_vec = []
  agent_manager_cost_vec = []
  construction_scene_manager_cost_vec = []
  reference_path_manager_cost_vec = []
  lateral_obstacle_cost_vec = []
  lane_tracks_mgr_cost_vec = []
  ConstructDynamicWorld_cost_vec = []
  edt_manager_cost_vec = []
  traffic_light_decider_cost_vec = []
  ego_lane_road_right_decider_cost_vec = []
  potential_dangerous_agent_decider_cost_vec = []
  lane_change_decider_cost_vec = []
  lat_lon_joint_planner_decider_cost_vec = []
  sample_poly_speed_adjust_decider_cost_vec = []
  lateral_obstacle_decider_cost_vec = []
  lane_borrow_decider_cost_vec = []
  lateral_offset_decider_cost_vec = []
  gap_selector_decider_cost_vec = []
  spatio_temporal_planner_cost_vec = []
  general_lateral_decider_cost_vec = []
  lateral_motion_planner_cost_vec = []
  stop_destination_decider_cost_vec = []
  mrc_brake_decider_cost_vec = []
  agent_longitudinal_decider_cost_vec = []
  construct_st_graph_cost_vec = []
  expand_st_boundaries_decider_cost_vec = []
  closest_in_path_vehicle_decider_cost_vec = []
  cipv_lost_prohibit_start_decider_cost_vec = []
  cipv_lost_prohibit_acceleration_decider_cost_vec = []
  parallel_longitudinal_avoid_decider_cost_vec = []
  agent_headway_decider_cost_vec = []
  longitudinal_decision_decider_cost_vec = []
  speed_limit_decider_cost_vec = []
  start_stop_decider_cost_vec = []
  long_ref_path_decider_cost_vec = []
  scc_longitudinal_motion_planner_cost_vec = []
  result_trajectory_generator_cost_vec = []
  hmi_decider_cost_vec = []
  planning_cost_time_vec = []
  log_cost_vec = []
  log_cost2_vec = []
  UpdateLDMap_cost = []
  CalculateRouteInfo_cost = []
  frame_nums = []
  for ind in range(len(bag_loader.plan_debug_msg['json'])):
    frame_nums.append(bag_loader.plan_debug_msg['data'][ind].frame_info.frame_num)
    SccLonBehaviorCostTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['SccLonBehaviorCostTime'], 2))
    SccLonMotionCostTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['SccLonMotionCostTime'], 2))
    SccLateralMotionCostTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['LateralMotionCostTime'], 2))
    SccLateralBehaviorCostTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['RealTimeLateralBehaviorCostTime'], 2))
    EnvironmentalModelManagerCost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['EnvironmentalModelManagerCost'], 2))
    GeneralPlannerModuleCostTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['GeneralPlannerModuleCostTime'], 2))
    DynamicWorldAverageCostTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['dynamic_world_cost'], 2))
    st_graph_searcher_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['st_graph_searcher_cost'], 2))
    ARAStarTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['Astar time'], 2))
    LateralMotionPlannerTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['LateralMotionPlannerTime'], 2))
    GeneralLateralDeciderCostTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['GeneralLateralDeciderCostTime'], 2))
    LateralObstacleDeciderTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['LateralObstacleDeciderTime'], 2))
    LaneChangeDeciderTime_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['LaneChangeDeciderTime'], 2))

    # 新增字段的读取逻辑（一一对应）
    TaskFunctionCost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['TaskFunctionCost'], 2))
    ego_state_update_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['ego_state_update cost'], 2))
    update_route_info_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['update route_info cost'], 2))
    virtual_lane_manager_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['virtual_lane_manager cost'], 2))
    traffic_light_decision_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['traffic_light_decision cost'], 2))
    obstacle_prediction_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['obstacle_prediction cost'], 2))
    obstacle_manager_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['obstacle_manager cost'], 2))
    agent_manager_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['agent_manager cost'], 2))
    construction_scene_manager_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['construction_scene_manager cost'], 2))
    reference_path_manager_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['reference_path_manager cost'], 2))
    lateral_obstacle_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['lateral_obstacle cost'], 2))
    lane_tracks_mgr_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['lane_tracks_mgr cost'], 2))
    ConstructDynamicWorld_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['ConstructDynamicWorld cost'], 2))
    edt_manager_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['edt_manager cost'], 2))
    traffic_light_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['traffic_light_decider_cost'], 2))
    ego_lane_road_right_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['ego_lane_road_right_decider_cost'], 2))
    potential_dangerous_agent_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['potential_dangerous_agent_decider_cost'], 2))
    lane_change_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['lane_change_decider_cost'], 2))
    lat_lon_joint_planner_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['lat_lon_joint_planner_decider_cost'], 2))
    sample_poly_speed_adjust_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['sample_poly_speed_adjust_decider_cost'], 2))
    lateral_obstacle_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['lateral_obstacle_decider_cost'], 2))
    lane_borrow_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['lane_borrow_decider_cost'], 2))
    lateral_offset_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['lateral_offset_decider_cost'], 2))
    gap_selector_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['gap_selector_decider_cost'], 2))
    spatio_temporal_planner_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['spatio_temporal_planner_cost'], 2))
    general_lateral_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['general_lateral_decider_cost'], 2))
    lateral_motion_planner_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['lateral_motion_planner_cost'], 2))
    stop_destination_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['stop_destination_decider_cost'], 2))
    mrc_brake_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['mrc_brake_decider_cost'], 2))
    agent_longitudinal_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['agent_longitudinal_decider_cost'], 2))
    construct_st_graph_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['construct_st_graph_cost'], 2))
    expand_st_boundaries_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['expand_st_boundaries_decider_cost'], 2))
    closest_in_path_vehicle_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['closest_in_path_vehicle_decider_cost'], 2))
    cipv_lost_prohibit_start_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['cipv_lost_prohibit_start_decider_cost'], 2))
    cipv_lost_prohibit_acceleration_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['cipv_lost_prohibit_acceleration_decider_cost'], 2))
    parallel_longitudinal_avoid_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['parallel_longitudinal_avoid_decider_cost'], 2))
    agent_headway_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['agent_headway_decider_cost'], 2))
    longitudinal_decision_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['longitudinal_decision_decider_cost'], 2))
    speed_limit_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['speed_limit_decider_cost'], 2))
    start_stop_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['start_stop_decider_cost'], 2))
    long_ref_path_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['long_ref_path_decider_cost'], 2))
    scc_longitudinal_motion_planner_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['scc_longitudinal_motion_planner_cost'], 2))
    result_trajectory_generator_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['result_trajectory_generator_cost'], 2))
    hmi_decider_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['hmi_decider_cost'], 2))
    planning_cost_time_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['planning_cost_time'], 2))
    log_cost_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['Log cost'], 2))
    log_cost2_vec.append(round(bag_loader.plan_debug_msg['json'][ind]['Log cost_2'], 2))
    UpdateLDMap_cost.append(round(bag_loader.plan_debug_msg['json'][ind]['UpdateLDMap cost'], 2))
    CalculateRouteInfo_cost.append(round(bag_loader.plan_debug_msg['json'][ind]['CalculateRouteInfo cost'], 2))
  frame_num_0 = frame_nums[0]
  frame_nums = [frame_num - frame_num_0 for frame_num in frame_nums]
  plan_debug_Astar.data.update({
  't_plan_debug': frame_nums,
  'st_graph_searcher_cost': st_graph_searcher_cost_vec,
  })

  cost_time_fig = bkp.figure(title='耗时',x_axis_label='time/s',
                  y_axis_label='time cost/(ms)', x_range = [frame_nums[0], frame_nums[-1]], width=1600,height=1600, output_backend="svg")
  cost_time_fig.line(frame_nums, SccLonBehaviorCostTime_vec, line_width=1, legend_label='LonBehaviorCostTime', color="red", visible=False)
  cost_time_fig.line(frame_nums, SccLonMotionCostTime_vec, line_width=1, legend_label='LonMotionCostTime_vec', color="blue", visible=False)
  cost_time_fig.line(frame_nums, SccLateralMotionCostTime_vec, line_width=1, legend_label='LatMotionCostTime_vec', color="orange", visible=False)
  cost_time_fig.line(frame_nums, EnvironmentalModelManagerCost_vec, line_width=1, legend_label='EnvironmentalCostTime_vec', color="yellow", visible=False)
  cost_time_fig.line(frame_nums, GeneralPlannerModuleCostTime_vec, line_width=1, legend_label='GeneralPlannerModuleCostTime', color="purple", visible=False)
  cost_time_fig.line(frame_nums, ARAStarTime_vec, line_width=1, legend_label='ARAStarTime_vec', color="khaki")
  cost_time_fig.line(frame_nums, GeneralLateralDeciderCostTime_vec, line_width=1, legend_label='GeneralLateralDeciderCostTime_vec', color="maroon")
  cost_time_fig.line(frame_nums, LateralMotionPlannerTime_vec, line_width=1, legend_label='LateralMotionPlannerTime_vec', color="gray")
  cost_time_fig.line(frame_nums, LateralObstacleDeciderTime_vec, line_width=1, legend_label='LateralObstacleDeciderTime_vec', color="brown")
  cost_time_fig.line(frame_nums, LaneChangeDeciderTime_vec, line_width=1, legend_label='LaneChangeDeciderTime_vec', color="black")

  # 新增字段绘图逻辑（为避免颜色重复，我用了不同色系，你可根据需要调整）
  # 第一组：环境/模型相关
  cost_time_fig.line(frame_nums, EnvironmentalModelManagerCost_vec, line_width=1, legend_label='EnvironmentalModelManagerCost', color="green")
  cost_time_fig.line(frame_nums, TaskFunctionCost_vec, line_width=1, legend_label='TaskFunctionCost', color="purple")
  cost_time_fig.line(frame_nums, ego_state_update_cost_vec, line_width=1, legend_label='ego_state_update cost', color="brown", visible=False)
  cost_time_fig.line(frame_nums, update_route_info_cost_vec, line_width=1, legend_label='update route_info cost', color="pink")
  cost_time_fig.line(frame_nums, virtual_lane_manager_cost_vec, line_width=1, legend_label='virtual_lane_manager cost', color="gray", visible=False)
  cost_time_fig.line(frame_nums, traffic_light_decision_cost_vec, line_width=1, legend_label='traffic_light_decision cost', color="olive", visible=False)
  cost_time_fig.line(frame_nums, obstacle_prediction_cost_vec, line_width=1, legend_label='obstacle_prediction cost', color="cyan")
  cost_time_fig.line(frame_nums, obstacle_manager_cost_vec, line_width=1, legend_label='obstacle_manager cost', color="magenta")
  cost_time_fig.line(frame_nums, agent_manager_cost_vec, line_width=1, legend_label='agent_manager cost', color="teal")
  cost_time_fig.line(frame_nums, construction_scene_manager_cost_vec, line_width=1, legend_label='construction_scene_manager cost', color="lavender", visible=False)
  cost_time_fig.line(frame_nums, reference_path_manager_cost_vec, line_width=1, legend_label='reference_path_manager cost', color="maroon", visible=True)
  cost_time_fig.line(frame_nums, lateral_obstacle_cost_vec, line_width=1, legend_label='lateral_obstacle cost', color="gold", visible=False)
  cost_time_fig.line(frame_nums, lane_tracks_mgr_cost_vec, line_width=1, legend_label='lane_tracks_mgr cost', color="coral", visible=False)
  cost_time_fig.line(frame_nums, ConstructDynamicWorld_cost_vec, line_width=1, legend_label='ConstructDynamicWorld cost', color="indigo", visible=False)
  cost_time_fig.line(frame_nums, edt_manager_cost_vec, line_width=1, legend_label='edt_manager cost', color="khaki", visible=False)

  # 第二组：决策相关
  cost_time_fig.line(frame_nums, traffic_light_decider_cost_vec, line_width=1, legend_label='traffic_light_decider_cost', color="navy", visible=False)
  cost_time_fig.line(frame_nums, ego_lane_road_right_decider_cost_vec, line_width=1, legend_label='ego_lane_road_right_decider_cost', color="salmon", visible=False)
  cost_time_fig.line(frame_nums, potential_dangerous_agent_decider_cost_vec, line_width=1, legend_label='potential_dangerous_agent_decider_cost', color="plum", visible=False)
  cost_time_fig.line(frame_nums, lane_change_decider_cost_vec, line_width=1, legend_label='lane_change_decider_cost', color="sienna", visible=False)
  cost_time_fig.line(frame_nums, lat_lon_joint_planner_decider_cost_vec, line_width=1, legend_label='lat_lon_joint_planner_decider_cost', color="tan")
  cost_time_fig.line(frame_nums, sample_poly_speed_adjust_decider_cost_vec, line_width=1, legend_label='sample_poly_speed_adjust_decider_cost', color="violet")
  cost_time_fig.line(frame_nums, lateral_obstacle_decider_cost_vec, line_width=1, legend_label='lateral_obstacle_decider_cost', color="turquoise")
  cost_time_fig.line(frame_nums, lane_borrow_decider_cost_vec, line_width=1, legend_label='lane_borrow_decider_cost', color="tomato")
  cost_time_fig.line(frame_nums, lateral_offset_decider_cost_vec, line_width=1, legend_label='lateral_offset_decider_cost', color="orchid", visible=False)
  cost_time_fig.line(frame_nums, gap_selector_decider_cost_vec, line_width=1, legend_label='gap_selector_decider_cost', color="peru", visible=False)
  cost_time_fig.line(frame_nums, spatio_temporal_planner_cost_vec, line_width=1, legend_label='spatio_temporal_planner_cost', color="palegreen")
  cost_time_fig.line(frame_nums, general_lateral_decider_cost_vec, line_width=1, legend_label='general_lateral_decider_cost', color="peachpuff")
  cost_time_fig.line(frame_nums, lateral_motion_planner_cost_vec, line_width=1, legend_label='lateral_motion_planner_cost', color="powderblue")
  cost_time_fig.line(frame_nums, stop_destination_decider_cost_vec, line_width=1, legend_label='stop_destination_decider_cost', color="rosybrown", visible=False)
  cost_time_fig.line(frame_nums, mrc_brake_decider_cost_vec, line_width=1, legend_label='mrc_brake_decider_cost', color="royalblue", visible=False)

  # 第三组：纵向决策/规划相关
  cost_time_fig.line(frame_nums, agent_longitudinal_decider_cost_vec, line_width=1, legend_label='agent_longitudinal_decider_cost', color="seagreen", visible=False)
  cost_time_fig.line(frame_nums, construct_st_graph_cost_vec, line_width=1, legend_label='construct_st_graph_cost', color="sienna")
  cost_time_fig.line(frame_nums, expand_st_boundaries_decider_cost_vec, line_width=1, legend_label='expand_st_boundaries_decider_cost', color="silver", visible=False)
  cost_time_fig.line(frame_nums, closest_in_path_vehicle_decider_cost_vec, line_width=1, legend_label='closest_in_path_vehicle_decider_cost', color="skyblue", visible=False)
  cost_time_fig.line(frame_nums, cipv_lost_prohibit_start_decider_cost_vec, line_width=1, legend_label='cipv_lost_prohibit_start_decider_cost', color="slateblue", visible=False)
  cost_time_fig.line(frame_nums, cipv_lost_prohibit_acceleration_decider_cost_vec, line_width=1, legend_label='cipv_lost_prohibit_acceleration_decider_cost', color="slategray", visible=False)
  cost_time_fig.line(frame_nums, parallel_longitudinal_avoid_decider_cost_vec, line_width=1, legend_label='parallel_longitudinal_avoid_decider_cost', color="snow", visible=False)
  cost_time_fig.line(frame_nums, agent_headway_decider_cost_vec, line_width=1, legend_label='agent_headway_decider_cost', color="springgreen", visible=False)
  cost_time_fig.line(frame_nums, longitudinal_decision_decider_cost_vec, line_width=1, legend_label='longitudinal_decision_decider_cost', color="steelblue", visible=False)
  cost_time_fig.line(frame_nums, speed_limit_decider_cost_vec, line_width=1, legend_label='speed_limit_decider_cost', color="tan", visible=False)
  cost_time_fig.line(frame_nums, start_stop_decider_cost_vec, line_width=1, legend_label='start_stop_decider_cost', color="thistle", visible=False)
  cost_time_fig.line(frame_nums, long_ref_path_decider_cost_vec, line_width=1, legend_label='long_ref_path_decider_cost', color="tomato", visible=False)
  cost_time_fig.line(frame_nums, scc_longitudinal_motion_planner_cost_vec, line_width=1, legend_label='scc_longitudinal_motion_planner_cost', color="turquoise", visible=False)
  cost_time_fig.line(frame_nums, result_trajectory_generator_cost_vec, line_width=1, legend_label='result_trajectory_generator_cost', color="violet", visible=False)
  cost_time_fig.line(frame_nums, hmi_decider_cost_vec, line_width=1, legend_label='hmi_decider_cost', color="wheat", visible=False)
  cost_time_fig.line(frame_nums, planning_cost_time_vec, line_width=1, legend_label='planning_cost_time_cost', color="wheat")
  cost_time_fig.line(frame_nums, log_cost_vec, line_width=1, legend_label='log_cost_vec', color="wheat")
  cost_time_fig.line(frame_nums, log_cost2_vec, line_width=1, legend_label='log_cost2_vec', color="wheat")
  cost_time_fig.line(frame_nums, UpdateLDMap_cost, line_width=1, legend_label='UpdateLDMap_cost', color="wheat")
  cost_time_fig.line(frame_nums, CalculateRouteInfo_cost, line_width=1, legend_label='CalculateRouteInfo_cost', color="wheat")
  f_cost_time = cost_time_fig.line("t_plan_debug", "st_graph_searcher_cost", source = plan_debug_Astar, line_width=1.6, legend_label='st_graph_searcher_cost', color="green")
  cost_time_fig.legend.click_policy = 'hide'

  hover_cost_time = HoverTool(renderers=[f_cost_time], tooltips=[('t_plan_debug','@t_plan_debug'),('st_graph_searcher_cost', '@st_graph_searcher_cost')], mode='vline')
  cost_time_fig.add_tools(hover_cost_time)
  cost_time_fig.toolbar.active_scroll = cost_time_fig.select_one(WheelZoomTool)

  return cost_time_fig