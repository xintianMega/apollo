/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file gridded_path_time_graph.cc
 **/

#include "modules/planning/tasks/optimizers/path_time_heuristic/gridded_path_time_graph.h"

#include <algorithm>
#include <limits>
#include <string>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"

#include "cyber/common/log.h"
#include "cyber/task/task.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::util::PointFactory;

namespace {

static constexpr double kDoubleEpsilon = 1.0e-6;

// Continuous-time collision check using linear interpolation as closed-loop
// dynamics
bool CheckOverlapOnDpStGraph(const std::vector<const STBoundary*>& boundaries,
                             const StGraphPoint& p1, const StGraphPoint& p2) {
  if (FLAGS_use_st_drivable_boundary) {
    return false;
  }
  for (const auto* boundary : boundaries) {
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    // Check collision between a polygon and a line segment
    if (boundary->HasOverlap({p1.point(), p2.point()})) {
      return true;
    }
  }
  return false;
}
}  // namespace

GriddedPathTimeGraph::GriddedPathTimeGraph(const StGraphData& st_graph_data,
const DpStSpeedOptimizerConfig& dp_config, const std::vector<const Obstacle*>& obstacles,
const common::TrajectoryPoint& init_point) : st_graph_data_(st_graph_data),
gridded_path_time_graph_config_(dp_config), obstacles_(obstacles), init_point_(init_point),
dp_st_cost_(dp_config, st_graph_data_.total_time_by_conf(), st_graph_data_.path_length(), obstacles,
st_graph_data_.st_drivable_boundary(), init_point_) {
  total_length_t_ = st_graph_data_.total_time_by_conf();
  unit_t_ = gridded_path_time_graph_config_.unit_t();
  total_length_s_ = st_graph_data_.path_length();
  dense_unit_s_ = gridded_path_time_graph_config_.dense_unit_s();
  sparse_unit_s_ = gridded_path_time_graph_config_.sparse_unit_s();
  dense_dimension_s_ = gridded_path_time_graph_config_.dense_dimension_s();
  // Safety approach preventing unreachable acceleration/deceleration
  max_acceleration_ = std::min(std::abs(vehicle_param_.max_acceleration()),
  std::abs(gridded_path_time_graph_config_.max_acceleration()));
  max_deceleration_ = -1.0 * std::min(std::abs(vehicle_param_.max_deceleration()),
  std::abs(gridded_path_time_graph_config_.max_deceleration()));
}

//构建st采样空间、cost值初始化、限速信息关联等数据准备的工作
/*1、遍历每个障碍物的boundry，判度是否有碰撞风险，如果有碰撞风险使用fallback速度规划；
2、初始化cost table
3、按照纵向采样点的s，查询各个位置处的限速
4、搜索可到达位置
5、计算可到达位置的cost
6、搜索最优路径*/
Status GriddedPathTimeGraph::Search(SpeedData* const speed_data) {
  static constexpr double kBounadryEpsilon = 1e-2;
  //遍历每个障碍物的boundry，判度是否有碰撞风险，如果有碰撞风险使用fallback速度规划；
  for (const auto& boundary : st_graph_data_.st_boundaries()) {
    // KeepClear obstacles not considered in Dp St decision
    /*如果boundary是KEEP_CLEAR类型，
    说明这个boundary不会影响车辆的行进，不予考虑，进行下一次循环*/
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    // If init point in collision with obstacle, return speed fallback
    // 如果起点在boundary内部，或十分接近，认为车辆此时不能移动，
    // 因此，在整个时间范围[0,t]内，设置v=0、a=0、s=0
    // ST图的起点，即车辆的初始位置，设定为(0,0)
    if (boundary->IsPointInBoundary({0.0, 0.0}) || (std::fabs(boundary->min_t()) < kBounadryEpsilon &&
    std::fabs(boundary->min_s()) < kBounadryEpsilon)) {
      dimension_t_ = static_cast<uint32_t>(std::ceil(total_length_t_ / static_cast<double>(unit_t_))) + 1;
      std::vector<SpeedPoint> speed_profile;
      double t = 0.0;
      for (uint32_t i = 0; i < dimension_t_; ++i, t += unit_t_) {
        speed_profile.push_back(PointFactory::ToSpeedPoint(0, t));
      }
      *speed_data = SpeedData(speed_profile);
      return Status::OK();
    }
  }

  /*初始化CostTable()
  1、时间维度是同质的，而空间维度有两种分辨率，稠密和稀疏，靠近自车的地方空间分布稠密；
  2、时间与空间分辨率检查
  3、计算coattable的时间尺度、空间上稀疏稠密的尺度，并且生成costtable
  4、按照时间、空间顺序来初始化costtable
  5、将空间上的采样点存入 spatial_distance_by_index_*/
  if (!InitCostTable().ok()) {
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /*查询限速信息，并且填入speed_limit_by_index_
  1、首先清空speed_limit_by_index_；
  2、然后从st_graph_data_ 从读取对应s处的限速值并填入*/
  if (!InitSpeedLimitLookUp().ok()) {
    const std::string msg = "Initialize speed limit lookup table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /*计算每个周期的位置取值范围以及该范围内各个位置采样点的cost值
  1、计算当前时间采样点位置s的可选范围内，每个点的cost值
  2、根据车辆运动以及障碍物boundary的限制，来确定在每个时间点t 内，从当前位置范围内出发，下个周期自车可以到达的位置s的 坐标范围；
  3、取所有位置s 采样点下一时刻可达到范围的集合输入给下一周期，用来做下一周期位置s采样点的范围。*/
  if (!CalculateTotalCost().ok()) {
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  /*速度动态规划
  使用动态规划求解最优st图上的速度路径，并将结果存入reference_line_info->mutable_speed_data()，
  即SpeedData* const speed_data 中；
  从indexs = 0, index_t = 7开始搜索，找到最后时刻cost最低的点*/
  if (!RetrieveSpeedProfile(speed_data).ok()) {
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

Status GriddedPathTimeGraph::InitCostTable() {
  // Time dimension is homogeneous while Spatial dimension has two resolutions,
  // dense and sparse with dense resolution coming first in the spatial horizon

  // Sanity check for numerical stability
  if (unit_t_ < kDoubleEpsilon) {
    const std::string msg = "unit_t is smaller than the kDoubleEpsilon.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Sanity check on s dimension setting
  if (dense_dimension_s_ < 1) {
    const std::string msg = "dense_dimension_s is at least 1.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  dimension_t_ = static_cast<uint32_t>(std::ceil(total_length_t_ / static_cast<double>(unit_t_))) + 1;

  double sparse_length_s = total_length_s_ - static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;
  sparse_dimension_s_ = sparse_length_s > std::numeric_limits<double>::epsilon()
  ? static_cast<uint32_t>(std::ceil(sparse_length_s / sparse_unit_s_)) : 0;
  dense_dimension_s_ = sparse_length_s > std::numeric_limits<double>::epsilon()
  ? dense_dimension_s_ : static_cast<uint32_t>(std::ceil(total_length_s_ / dense_unit_s_)) + 1;
  dimension_s_ = dense_dimension_s_ + sparse_dimension_s_;

  // Sanity Check
  if (dimension_t_ < 1 || dimension_s_ < 1) {
    const std::string msg = "Dp st cost table size incorrect.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  cost_table_ = std::vector<std::vector<StGraphPoint>>(dimension_t_,
  std::vector<StGraphPoint>(dimension_s_, StGraphPoint()));

  double curr_t = 0.0;
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {
    auto& cost_table_i = cost_table_[i];
    double curr_s = 0.0;
    for (uint32_t j = 0; j < dense_dimension_s_; ++j, curr_s += dense_unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
    curr_s = static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_ + sparse_unit_s_;
    for (uint32_t j = dense_dimension_s_; j < cost_table_i.size(); ++j, curr_s += sparse_unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
  }

  const auto& cost_table_0 = cost_table_[0];
  spatial_distance_by_index_ = std::vector<double>(cost_table_0.size(), 0.0);
  for (uint32_t i = 0; i < cost_table_0.size(); ++i) {
    spatial_distance_by_index_[i] = cost_table_0[i].point().s();
  }
  return Status::OK();
}

Status GriddedPathTimeGraph::InitSpeedLimitLookUp() {
  speed_limit_by_index_.clear();

  speed_limit_by_index_.resize(dimension_s_);
  const auto& speed_limit = st_graph_data_.speed_limit();

  for (uint32_t i = 0; i < dimension_s_; ++i) {
    speed_limit_by_index_[i] = speed_limit.GetSpeedLimitByS(cost_table_[0][i].point().s());
  }
  return Status::OK();
}

Status GriddedPathTimeGraph::CalculateTotalCost() {
  // col and row are for STGraph
  // t corresponding to col
  // s corresponding to row
  size_t next_highest_row = 0;
  size_t next_lowest_row = 0;

  for (size_t c = 0; c < cost_table_.size(); ++c) {
    size_t highest_row = 0;
    size_t lowest_row = cost_table_.back().size() - 1;

    int count = static_cast<int>(next_highest_row) - static_cast<int>(next_lowest_row) + 1;
    if (count > 0) {
      std::vector<std::future<void>> results;
      for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
        auto msg = std::make_shared<StGraphMessage>(c, r);
        if (FLAGS_enable_multi_thread_in_dp_st_graph) {
          results.push_back(cyber::Async(&GriddedPathTimeGraph::CalculateCostAt, this, msg));
        } else {
          /* 采样点的cost计算
          计算采样点的cost方法在CalculateCostAt(msg) 方法中，msg中由(c,r)两个变量组成，c代表时间采样点，r代表距离采样点
          根据c、r读取 cost_table_ 中的对应点
          根据tObstacleCost、SpatialPotentialCost、CalculateEdgeCostForSecondCol等多个方面来计算cost值，
          同时第1、第2、第3个时间序列需要特殊处理*/
          CalculateCostAt(msg);
        }
      }
      if (FLAGS_enable_multi_thread_in_dp_st_graph) {
        for (auto& result : results) {
          result.get();
        }
      }
    }

    // 下一轮循环的准备工作
    // 遍历当前列中行号在[next_lowest_row,next_highest_row] 的点，
    // 找到 highest_row 和 lowest_row，
    // 以此来更新下一轮循环需要的next_highest_row、next_lowest_row
    for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
      const auto& cost_cr = cost_table_[c][r];
      if (cost_cr.total_cost() < std::numeric_limits<double>::infinity()) {
        size_t h_r = 0;
        size_t l_r = 0;
        //当前时刻当前位置采样点，下一时刻可到达范围的计算GetRowRange()
        GetRowRange(cost_cr, &h_r, &l_r);
        highest_row = std::max(highest_row, h_r);
        lowest_row = std::min(lowest_row, l_r);
      }
    }
    next_highest_row = highest_row;
    next_lowest_row = lowest_row;
  }

  return Status::OK();
}

//找K的范围，依据max_acceleration_和max_deceleration_的剪枝操作
void GriddedPathTimeGraph::GetRowRange(const StGraphPoint& point, size_t* next_highest_row,
size_t* next_lowest_row) {
  double v0 = 0.0;
  // (all): Record speed information in StGraphPoint and deprecate this.
  // A scaling parameter for DP range search due to the lack of accurate
  // information of the current velocity (set to 1 by default since we use
  // past 1 second's average v as approximation)
  double acc_coeff = 0.5;
  if (!point.pre_point()) {
    v0 = init_point_.v();
  } else {
    v0 = point.GetOptimalSpeed();
  }

  const auto max_s_size = dimension_s_ - 1;
  const double t_squared = unit_t_ * unit_t_;
  const double s_upper_bound = v0 * unit_t_ + acc_coeff * max_acceleration_ * t_squared + point.point().s();
  const auto next_highest_itr = std::lower_bound(spatial_distance_by_index_.begin(),
  spatial_distance_by_index_.end(), s_upper_bound);
  if (next_highest_itr == spatial_distance_by_index_.end()) {
    *next_highest_row = max_s_size;
  } else {
    *next_highest_row = std::distance(spatial_distance_by_index_.begin(), next_highest_itr);
  }

  const double s_lower_bound = std::fmax(0.0, v0 * unit_t_ + acc_coeff * max_deceleration_ * t_squared) + point.point().s();
  const auto next_lowest_itr = std::lower_bound(spatial_distance_by_index_.begin(), spatial_distance_by_index_.end(), s_lower_bound);
  if (next_lowest_itr == spatial_distance_by_index_.end()) {
    *next_lowest_row = max_s_size;
  } else {
    *next_lowest_row = std::distance(spatial_distance_by_index_.begin(), next_lowest_itr);
  }
}

/*计算每一个点的totalCost，并更新父节点
 * 输入 msg，即(c,r)，cost_table_中t、s方向离散后的index
 *    一个点的total_cost由四部分构成：
 *    1、障碍物cost                  obstacle_cost
 *    2、空间位置cost，于终点s的差值  spatial_potential_cost
 *    3、前一个点的total_cost
 *    4、EdgeCost，由三部分构成      Speedcost、AccelCost、JerkCost*/
void GriddedPathTimeGraph::CalculateCostAt(const std::shared_ptr<StGraphMessage>& msg) {
  const uint32_t c = msg->c;
  const uint32_t r = msg->r;
  auto& cost_cr = cost_table_[c][r];

  // 1、计算 obstacle_cost，如果为无穷大，则停止
  cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));
  if (cost_cr.obstacle_cost() > std::numeric_limits<double>::max()) {
    return;
  }

  // 2、计算SpatialPotentialCost
  cost_cr.SetSpatialPotentialCost(dp_st_cost_.GetSpatialPotentialCost(cost_cr));

  // 第0列的特殊处理，设置起始点的TotalCost 为0
  const auto& cost_init = cost_table_[0][0];
  if (c == 0) {
    DCHECK_EQ(r, 0U) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0);
    cost_cr.SetOptimalSpeed(init_point_.v());
    return;
  }

  const double speed_limit = speed_limit_by_index_[r];
  const double cruise_speed = st_graph_data_.cruise_speed();
  // The mininal s to model as constant acceleration formula default: 0.25 * 7 = 1.75 m
  const double min_s_consider_speed = dense_unit_s_ * dimension_t_;

  if (c == 1) {
    //当前点的加速度
    const double acc = 2 * (cost_cr.point().s() / unit_t_ - init_point_.v()) / unit_t_;
    if (acc < max_deceleration_ || acc > max_acceleration_) {
      return;
    }

    if (init_point_.v() + acc * unit_t_ < -kDoubleEpsilon && cost_cr.point().s() > min_s_consider_speed) {
      return;
    }

    // 当前点与起始点的连线与stboundary有重叠，返回
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr, cost_init)) {
      return;
    }

    // 计算当前点的total_cost
    cost_cr.SetTotalCost(cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
    cost_init.total_cost() + CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed));
    cost_cr.SetPrePoint(cost_init);
    cost_cr.SetOptimalSpeed(init_point_.v() + acc * unit_t_);
    return;
  }

  // 剪枝操作
  static constexpr double kSpeedRangeBuffer = 0.20;
  // 由当前点推出能到达该点的前一列最小的s,将当前点的pre_col缩小至 [r_low, r]
  const double pre_lowest_s = cost_cr.point().s() -
  FLAGS_planning_upper_speed_limit * (1 + kSpeedRangeBuffer) * unit_t_;
  const auto pre_lowest_itr = std::lower_bound(spatial_distance_by_index_.begin(),
  spatial_distance_by_index_.end(), pre_lowest_s);
  uint32_t r_low = 0;
  if (pre_lowest_itr == spatial_distance_by_index_.end()) {
    r_low = dimension_s_ - 1;
  } else {
    r_low = static_cast<uint32_t>(std::distance(spatial_distance_by_index_.begin(), pre_lowest_itr));
  }
  const uint32_t r_pre_size = r - r_low + 1;
  const auto& pre_col = cost_table_[c - 1];
  double curr_speed_limit = speed_limit;

  // 第二列的特殊处理
  if (c == 2) {
    // 对于前一列，遍历从r->r_low的点，
    // 依据重新算得的cost，当前点的pre_point，也就是DP过程的状态转移方程
    for (uint32_t i = 0; i < r_pre_size; ++i) {
      uint32_t r_pre = r - i;
      if (std::isinf(pre_col[r_pre].total_cost()) || pre_col[r_pre].pre_point() == nullptr) {
        continue;
      }
      // (Jiaxuan): Calculate accurate acceleration by recording speed
      // data in ST point.
      // Use curr_v = (point.s - pre_point.s) / unit_t as current v
      // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
      // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
      // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
      const double curr_a = 2 * ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
      pre_col[r_pre].GetOptimalSpeed()) / unit_t_;
      if (curr_a < max_deceleration_ || curr_a > max_acceleration_) {
        continue;
      }

      if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
      cost_cr.point().s() > min_s_consider_speed) {
        continue;
      }

      // Filter out continuous-time node connection which is in collision with obstacle
      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr, pre_col[r_pre])) {
        continue;
      }
      curr_speed_limit = std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
      const double cost = cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
      pre_col[r_pre].total_cost() + CalculateEdgeCostForThirdCol( r, r_pre, curr_speed_limit, cruise_speed);

      if (cost < cost_cr.total_cost()) {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]);
        cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_);
      }
    }
    return;
  }

  for (uint32_t i = 0; i < r_pre_size; ++i) {
    uint32_t r_pre = r - i;
    if (std::isinf(pre_col[r_pre].total_cost()) || pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }
    // Use curr_v = (point.s - pre_point.s) / unit_t as current v
    // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
    // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
    // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
    const double curr_a = 2 *((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
    pre_col[r_pre].GetOptimalSpeed()) / unit_t_;
    if (curr_a > max_acceleration_ || curr_a < max_deceleration_) {
      continue;
    }

    if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
    cost_cr.point().s() > min_s_consider_speed) {
      continue;
    }

    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr, pre_col[r_pre])) {
      continue;
    }

    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }

    if (!prepre_graph_point.pre_point()) {
      continue;
    }
    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();
    const STPoint& prepre_point = prepre_graph_point.point();
    const STPoint& pre_point = pre_col[r_pre].point();
    const STPoint& curr_point = cost_cr.point();
    curr_speed_limit = std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
    double cost = cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
    pre_col[r_pre].total_cost() + CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
    curr_point, curr_speed_limit, cruise_speed);

    if (cost < cost_cr.total_cost()) {
      cost_cr.SetTotalCost(cost);
      cost_cr.SetPrePoint(pre_col[r_pre]);
      cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_);
    }
  }
}

/*遍历每一列的最后一个点，找正在的best_end_point，并更新min_cost
这里不直接使用最后一列的min_cost点作为终点
因为采样时间是一个预估时间窗口，在这之前可能就到达终点了*/
Status GriddedPathTimeGraph::RetrieveSpeedProfile(SpeedData* const speed_data) {
  double min_cost = std::numeric_limits<double>::infinity();
  const StGraphPoint* best_end_point = nullptr;
  for (const StGraphPoint& cur_point : cost_table_.back()) {
    if (!std::isinf(cur_point.total_cost()) && cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  // 遍历每一列的最后一个点，找正在的best_end_point，并更新min_cost
  // 这里不直接使用最后一列的min_cost点作为终点
  // 因为采样时间是一个预估时间窗口，在这之前可能就到达终点了
  for (const auto& row : cost_table_) {
    const StGraphPoint& cur_point = row.back();
    if (!std::isinf(cur_point.total_cost()) && cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 回溯，得到最优的 speed_profile
  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_point;
  while (cur_point != nullptr) {
    ADEBUG << "Time: " << cur_point->point().t();
    ADEBUG << "S: " << cur_point->point().s();
    ADEBUG << "V: " << cur_point->GetOptimalSpeed();
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_profile.push_back(speed_point);
    cur_point = cur_point->pre_point();
  }
  std::reverse(speed_profile.begin(), speed_profile.end());

  static constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  if (speed_profile.front().t() > kEpsilon || speed_profile.front().s() > kEpsilon) {
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 计算每个点的速度 v
  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    const double v = (speed_profile[i + 1].s() - speed_profile[i].s()) /
    (speed_profile[i + 1].t() - speed_profile[i].t() + 1e-3);
    speed_profile[i].set_v(v);
  }

  *speed_data = SpeedData(speed_profile);
  return Status::OK();
}

double GriddedPathTimeGraph::CalculateEdgeCost(const STPoint& first, const STPoint& second, const STPoint& third,
const STPoint& forth, const double speed_limit, const double cruise_speed) {
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit, cruise_speed) +
  dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
  dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

double GriddedPathTimeGraph::CalculateEdgeCostForSecondCol(const uint32_t row,
const double speed_limit, const double cruise_speed) {
  double init_speed = init_point_.v();
  double init_acc = init_point_.a();
  const STPoint& pre_point = cost_table_[0][0].point();
  const STPoint& curr_point = cost_table_[1][row].point();
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit, cruise_speed) +
  dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point, curr_point) +
  dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point, curr_point);
}

double GriddedPathTimeGraph::CalculateEdgeCostForThirdCol(const uint32_t curr_row,
const uint32_t pre_row, const double speed_limit, const double cruise_speed) {
  double init_speed = init_point_.v();
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit, cruise_speed) +
  dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
  dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}

}  // namespace planning
}  // namespace apollo
