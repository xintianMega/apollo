/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planner/planner_dispatcher.h"

#include <memory>

#include "modules/planning/planner/lattice/lattice_planner.h"
#include "modules/planning/planner/navi/navi_planner.h"
#include "modules/planning/planner/public_road/public_road_planner.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

void PlannerDispatcher::RegisterPlanners() {
  // rtk - 根据录制的轨迹来规划行车路线
  planner_factory_.Register(
      PlannerType::RTK,
      [](const std::shared_ptr<DependencyInjector>& injector) -> Planner* {
        return new RTKReplayPlanner(injector);
      });

  // public_road - 开放道路的轨迹规划器
  //open_space - 自主泊车规划器
  planner_factory_.Register(
      PlannerType::PUBLIC_ROAD,
      [](const std::shared_ptr<DependencyInjector>& injector) -> Planner* {
        return new PublicRoadPlanner(injector);
      });

  // lattice - 基于网格算法的轨迹规划器
  planner_factory_.Register(
      PlannerType::LATTICE,
      [](const std::shared_ptr<DependencyInjector>& injector) -> Planner* {
        return new LatticePlanner(injector);
      });

  //navi - 基于实时相对地图的规划器
  planner_factory_.Register(
      PlannerType::NAVI,
      [](const std::shared_ptr<DependencyInjector>& injector) -> Planner* {
        return new NaviPlanner(injector);
      });
}

}  // namespace planning
}  // namespace apollo
