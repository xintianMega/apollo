# LaneFollow场景规划

## 1. LaneChangeDecider

LaneChangeDecider 是lanefollow 场景下，所调用的第一个task，它的作用主要有两点：

- 判断当前是否进行变道，以及变道的状态，并将结果存在变量lane_change_status中；
- 变道过程中将目标车道的reference line放置到首位，变道结束后将当前新车道的reference line放置到首位

## 2. PathReuseDecider

PathReuseDecider 是lanefollow 场景下，所调用的第 2 个 task，它的作用主要是换道时：

- 根据横纵向跟踪偏差，来决策是否需要重新规划轨迹；
- 如果横纵向跟踪偏差，则根据上一时刻的轨迹生成当前周期的轨迹，以尽量保持轨迹的一致性

## 3. PathLaneBorrowDecider

PathLaneBorrowDecider 是第3个task，PathLaneBorrowDecider会判断已处于借道场景下判断是否退出避让；判断未处于借道场景下判断是否具备借道能力。

需要满足下面条件才能判断是否可以借道：

- 只有一条参考线，才能借道
- 起点速度小于最大借道允许速度
- 阻塞障碍物必须远离路口
- 阻塞障碍物会一直存在
- 阻塞障碍物与终点位置满足要求
- 为可侧面通过的障碍物

## 4. PathBoundsDecider

PathBoundsDecider 是第四个task，PathBoundsDecider根据lane borrow决策器的输出、本车道以及相邻车道的宽度、障碍物的左右边界，来计算path 的boundary，从而将path 搜索的边界缩小，将复杂问题转化为凸空间的搜索问题，方便后续使用QP算法求解。



## 5. PiecewiseJerkPathOptimizer

PiecewiseJerkPathOptimizer 是lanefollow 场景下，所调用的第 5 个 task，属于task中的optimizer类别它的作用主要是：

- 根据之前decider决策的reference line和 path bound，以及横向约束，将最优路径求解问题转化为二次型规划问题；
- 调用osqp库求解最优路径；

## 6. PathAssessmentDecider

PathAssessmentDecider 是lanefollow 场景下，所调用的第 6 个 task，属于task中的decider 类别它的作用主要是：

- 选出之前规划的备选路径中排序最靠前的路径；
- 添加一些必要信息到路径中

## 7. PathDecider

PathDecider 是lanefollow 场景下，所调用的第 7 个 task，属于task中的decider 类别它的作用主要是：

在上一个任务中获得了最优的路径，PathDecider的功能是根据静态障碍物做出自车的决策，对于前方的静态障碍物是忽略、stop还是nudge

## 8. RuleBasedStopDecider

RuleBasedStopDecider 是lanefollow 场景下，所调用的第 8 个 task，属于task中的decider 类别它的作用主要是：

根据一些规则来设置停止标志。

## 9. SPEED_BOUNDS_PRIORI_DECIDER

SPEED_BOUNDS_PRIORI_DECIDER 是lanefollow 场景下，所调用的第 10 个 task，属于task中的decider 类别它的作用主要是：

- 将规划路径上障碍物的st bounds 加载到路径对应的st 图上

- 计算并生成路径上的限速信息

## 10. PathTimeHeuristicOptimizer

SPEED_HEURISTIC_OPTIMIZER 是lanefollow 场景下，所调用的第 11个 task，属于task中的optimizer 类别，它的作用主要是：

- apollo中使用动态规划的思路来进行速度规划，其实更类似于使用动态规划的思路进行速度决策；

- 首先将st图进行网格化，然后使用动态规划求解一条最优路径，作为后续进一步速度规划的输入，将问题的求解空间转化为凸空间

代码总流程如下：

- 遍历每个障碍物的boundry，判度是否有碰撞风险，如果有碰撞风险使用fallback速度规划；
- 初始化cost table
- 按照纵向采样点的s，查询各个位置处的限速
- 搜索可到达位置
- 计算可到达位置的cost
- 搜索最优路径

## 11. SpeedDecider

SpeedDecider 是lanefollow 场景下，Apollo Planning算法所调用的第12个 task，属于task中的decider 类别它的作用主要是：

- 对每个目标进行遍历，分别对每个目标进行决策
- 或得mutable_obstacle->path_st_boundary()
- 根据障碍物st_boundary的时间与位置的分布，判断是否要忽略
- 对于虚拟目标 Virtual obstacle，如果不在referenceline的车道上，则跳过
- 如果是行人则决策结果置为stop
- SpeedDecider::GetSTLocation() 获取障碍物在st图上与自车路径的位置关系
- 根据不同的STLocation，来对障碍物进行决策
- 如果没有纵向决策结果，则置位ignore_decision

## 12. SPEED_BOUNDS_FINAL_DECIDER

SPEED_BOUNDS_FINAL_DECIDER 是lanefollow 场景下，所调用的第 13 个 task，属于task中的decider 类别它的作用主要是：

- 将规划路径上障碍物的st bounds 加载到路径对应的st 图上
- 计算并生成路径上的限速信息

## 13. PiecewiseJerkSpeedOptimizer

PiecewiseJerkSpeedOptimizer 是lanefollow 场景下，所调用的第 14个 task，属于task中的decider 类别它的作用主要是：

- 根据之前decider决策的speed decider和 speed bound，以及纵向约束，将最优速度求解问题转化为二次型规划问题；
- 调用osqp库求解最优路径