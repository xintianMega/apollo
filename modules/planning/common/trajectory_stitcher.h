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
 * @file
 **/

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class TrajectoryStitcher {
 public:
  TrajectoryStitcher() = delete;

  //把上一时刻的规划轨迹 PublishableTrajectory 中的每个点进行坐标转换(自车位置变化, 转换坐标在两帧自车坐标系中).
  static void TransformLastPublishedTrajectory(
      const double x_diff, const double y_diff, const double theta_diff,
      PublishableTrajectory* prev_trajectory);

  /*核心成员函数
  负责决定哪些情况下需要 Replan,
  随后进入 ComputeReinitStitchingTrajectory 成员函数.
  不 Replan 的话, 截取上次轨迹的一段作为规划轨迹发出去.
  */
  static std::vector<common::TrajectoryPoint> ComputeStitchingTrajectory(
      const common::VehicleState& vehicle_state, const double current_timestamp,
      const double planning_cycle_time, const size_t preserved_points_num,
      const bool replan_by_offset, const PublishableTrajectory* prev_trajectory,
      std::string* replan_reason);

  /*
  返回值 std::vector<common::TrajectoryPoint> 中只有一个点, 即 Replan 规划起始点.
  自车速度与加速度较小时, 不需要航迹推算, 直接用 ComputeTrajectoryPointFromVehicleState
  自车当前位置点作为 Replan 规划起始点即可(此处有疑问: 对于泊车尤其是狭小空间内的泊车, 这个思路误差有可能会不会太大).
  否则利用 VehicleModel::Predict(planning_cycle_time, vehicle_state) 函数进行航迹推算得到 Replan 规划起始点.
  */
  static std::vector<common::TrajectoryPoint> ComputeReinitStitchingTrajectory(
      const double planning_cycle_time,
      const common::VehicleState& vehicle_state);

 private:
  static std::pair<double, double> ComputePositionProjection(
      const double x, const double y,
      const common::TrajectoryPoint& matched_trajectory_point);

  static common::TrajectoryPoint ComputeTrajectoryPointFromVehicleState(
      const double planning_cycle_time,
      const common::VehicleState& vehicle_state);
};

}  // namespace planning
}  // namespace apollo
