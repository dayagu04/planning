#include "ssc_map.h"
#include <cstddef>
#include "define/geometry.h"
#include "log.h"

namespace planning {

SscMap::SscMap(const SscMap::Config &config) : config_(config) {

  p_3d_grid_ = new GridMapND<SscMapDataType, 3>(
      config_.map_size, config_.map_resolution, config_.axis_name);
  p_3d_inflated_grid_ = new GridMapND<SscMapDataType, 3>(
      config_.map_size, config_.map_resolution, config_.axis_name);
}

void SscMap::ResetSscMap(
    const FrenetEgoState &ego_frenet_state, const double &time) {
  ClearGridMap();
  UpdateMapOrigin(ego_frenet_state, time);

  return;
}

void SscMap::UpdateMapOrigin(
    const FrenetEgoState &ego_state, const double &time) {
  std::array<double, 3> map_origin;
  // set s
  map_origin[0] = ego_state.s() - config_.s_back_len;
  //set l
  map_origin[1] = 0.0;
  //set t
  map_origin[2] = time;

  p_3d_grid_->set_origin(map_origin);
  p_3d_inflated_grid_->set_origin(map_origin);
}

void SscMap::ConstructSscMap(
    const AgentFrenetSpatioTemporalInFo
      &surround_trajs_state_info) {
  p_3d_grid_->clear_data();
  p_3d_inflated_grid_->clear_data();

  // FillStaticPart(obstacle_grids);
  FillDynamicPart(surround_trajs_state_info);

  return;
}

void SscMap::ClearGridMap() {
  p_3d_grid_->clear_data();
  p_3d_inflated_grid_->clear_data();
  return;
}

// void SscMap::FillStaticPart(const std::vector<AgentFrenetSpatioTemporalInFo>) {
//   for (int i = 0; i < static_cast<int>(obs_grid_fs.size()); ++i) {
//     if (obs_grid_fs[i](0) <= 0) {
//       continue;
//     }
//     for (int k = 0; k < config_.map_size[2]; ++k) {
//       std::array<decimal_t, 3> pt = {{obs_grid_fs[i](0), obs_grid_fs[i](1),
//                                       (double)k * config_.map_resolution[2]}};
//       auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(pt);
//       if (p_3d_grid_->CheckCoordInRange(coord)) {
//         p_3d_grid_->SetValueUsingCoordinate(coord, 100);
//       }
//     }
//   }
//   return;
// }

void SscMap::FillDynamicPart(
  const AgentFrenetSpatioTemporalInFo &sur_vehicle_trajs_fs) {
  // FillMapWithFsVehicleTraj(sur_vehicle_trajs_fs.frenet_vertices);
  return;
}

void SscMap::FillMapWithFsVehicleTraj(
    const std::vector<std::vector<SLTPoint>> &traj_point) {
  if (traj_point.size() == 0) {
    LOG_DEBUG("FillMapWithFsVehicleTraj::Trajectory is empty.");
    return;
  }
  for (size_t i = 0; i < traj_point.size(); ++i) {
    bool is_valid = true;
    for (const auto& v : traj_point[i]) {
      if (v.s() <= 0) {
        is_valid = false;
        break;
      }
    }
    if (!is_valid) {
      continue;
    }
    double z = traj_point[i][0].t();
    int t_idx = 0;
    std::vector<planning::Point2i> v_coord;
    std::array<double, 3> p_w;
    for (const auto &v : traj_point[i]) {
      p_w = {v.s(), v.l(), v.t()};
      auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(p_w);
      t_idx = coord[2];
      if (!p_3d_grid_->CheckCoordInRange(coord)) {
        is_valid = false;
        break;
      }
      v_coord.push_back(planning::Point2i(coord[0], coord[1]));
    }
    if (!is_valid) {
      continue;
    }
    // std::vector<std::vector<cv::Point2i>> vv_coord_cv;
    // std::vector<cv::Point2i> v_coord_cv;
    // GetCvPoint2iVecUsingCommonPoint2iVec(v_coord, &v_coord_cv);
    // vv_coord_cv.push_back(v_coord_cv);
    // int w = p_3d_grid_->dims_size()[0];
    // int h = p_3d_grid_->dims_size()[1];
    // int layer_offset = t_idx * w * h;
    // cv::Mat layer_mat =
    //     cv::Mat(h, w, CV_MAKETYPE(cv::DataType<SscMapDataType>::type, 1),
    //             p_3d_grid_->get_data_ptr() + layer_offset);
    // cv::fillPoly(layer_mat, vv_coord_cv, 100);
  }

  return;
}

// void SscMap::GetCvPoint2iVecUsingCommonPoint2iVec(
//     const std::vector<Point2i>& pts_in, std::vector<cv::Point2i>* pts_out) {
//   int num = pts_in.size();
//   pts_out->resize(num);
//   for (int i = 0; i < num; ++i) {
//     GetCvPoint2iUsingCommonPoint2i(pts_in[i], pts_out->data() + i);
//   }
//   return;
// }

// void SscMap::GetCvPoint2iUsingCommonPoint2i(const Point2i& pt_in,
//                                             cv::Point2i* pt_out) {
//   pt_out->x = pt_in.x;
//   pt_out->y = pt_in.y;
//   return;
// }

}  // namespace planning