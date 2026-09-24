# 混合 A* 算法代码解析

## 1. 项目中实际使用的是哪一个实现

本项目面向四轮非完整约束车辆的局部路径搜索由 `fast_planner::KinodynamicAstar2D` 完成。虽然类名是 `KinodynamicAstar2D`，但从搜索状态、运动模型和终点解析连接来看，它实现的是车辆领域常见的混合 A* 思路：

- A* 的节点管理在离散的 `(x_index, y_index, yaw_index)` 网格上进行；
- 节点内部保存连续的 `(x, y, yaw, speed)` 状态；
- 相邻节点不是普通栅格 A* 的直线八邻域，而是自行车模型积分得到的连续曲线运动原语；
- 终点附近尝试用满足最小转弯半径约束的 Dubins 曲线直接连接。

主要文件如下：

| 作用 | 文件或函数 |
| --- | --- |
| 类声明、搜索接口 | `src/TGH-Planner/TGH_Planner/path_searching/include/path_searching/kinodynamic_astar_2D.h` |
| 混合 A* 主体 | `src/TGH-Planner/TGH_Planner/path_searching/src/kinodynamic_astar_2D.cpp` |
| 节点、哈希表、优先队列比较器 | `src/TGH-Planner/TGH_Planner/path_searching/include/path_searching/kinodynamic_astar.h` |
| 创建搜索器并调用搜索 | `src/TGH-Planner/TGH_Planner/plan_manage/src/planner_manager.cpp` |
| ROS 参数 | `src/TGH-Planner/TGH_Planner/plan_manage/launch/kino_algorithm.xml` |

三维无人机版 `KinodynamicAstar` 和普通二维栅格版 `Astar2D` 也保留在仓库中，但当前 `FastPlannerManager` 创建的是 `KinodynamicAstar2D`，因此本文以它为准。

## 2. 算法在整个规划系统中的位置

调用链为：

```text
占据栅格 / ESDF 地图
          │
          ├── TopologyPRM::findGuidePath()
          │          生成拓扑 Guide Path
          ▼
FastPlannerManager::kinodynamicReplan()
          │
          ├── reset()
          ├── setGuidePath(guide_path_last)
          ├── search(start..., end...)
          └── getKinoTraj(delta_t)
                     │
                     ▼
            离散初始轨迹点
                     │
                     ▼
    NonUniformBspline::parameterizeToBspline()
                     │
                     ▼
             B-spline 优化与执行
```

因此，混合 A* 位于“拓扑路径生成”和“B 样条优化”之间。拓扑路径提供搜索方向，混合 A* 将其转化为满足车辆转向约束且避障的初始轨迹，B 样条模块再对轨迹进行连续化和优化。

Guide Path 是软引导而不是硬约束：它只参与启发函数计算，并不要求最终轨迹逐点经过 Guide Path。没有新 Guide Path 时，`planner_manager.cpp` 会继续使用上一次保存的路径。

## 3. 输入是什么

混合 A* 的输入分为显式搜索参数和搜索器预先持有的上下文。

### 3.1 `search()` 的显式输入

```cpp
int search(Eigen::Vector3d start_pt,
           Eigen::Vector3d start_vel,
           Eigen::Vector3d start_acc,
           double start_yaw,
           Eigen::Vector3d end_pt,
           Eigen::Vector3d end_vel,
           double end_yaw,
           bool init,
           bool dynamic = false,
           double time_start = -1.0,
           bool gen_search = true);
```

| 参数 | 含义 | 在当前二维实现中的用途 |
| --- | --- | --- |
| `start_pt` | 起点 `(x,y,z)` | `x,y` 参与搜索，`z` 保存为输出轨迹的固定高度 |
| `start_vel` | 起点速度向量 | 模长作为起点标量速度；同时用于 B 样条起始导数 |
| `start_acc` | 起点加速度 | 不参与节点扩展，保存给后续 B 样条边界条件 |
| `start_yaw` | 起点航向角，rad | 写入搜索状态 |
| `end_pt` | 目标点 `(x,y,z)` | `x,y` 作为目标位置 |
| `end_vel` | 目标速度向量 | 模长写入目标状态；当前启发函数主要使用位置和航向 |
| `end_yaw` | 目标航向角，rad | 用于目标状态及 Dubins 直连 |
| `init` | 是否保持初始运动连续性 | 为 `true` 时第一轮只以当前速度、零转角向前扩展；失败后管理器以 `false` 重试 |
| `dynamic` | 是否把时间离散索引加入节点键 | 当前调用使用默认 `false` |
| `time_start` | 动态搜索起始时刻 | 仅 `dynamic=true` 时使用 |
| `gen_search` | 预留标志 | 当前函数体没有使用 |

### 3.2 搜索器上下文输入

1. **ESDF/占据地图**：通过 `setEnvironment()` 注入。每条运动原语和 Dubins 直连段都调用二维距离查询，要求障碍距离不小于 `obs_dis_`。
2. **拓扑 Guide Path**：通过 `setGuidePath()` 注入。代码假定它是稠密离散路径，并建立 PCL KdTree 查询当前状态离路径最近的点。
3. **车辆和搜索参数**：由 `setParam()` 从 ROS 参数服务器读取，例如最大速度、最大加速度、轴距、最大转角、搜索分辨率、航向分辨率、搜索视距和启发函数权重。

## 4. 输出是什么

### 4.1 `search()` 的直接输出：搜索状态码

| 状态码 | 数值 | 含义 |
| --- | ---: | --- |
| `REACH_HORIZON` | 1 | 搜索到局部规划视距 `horizon_`，返回一段可用局部路径 |
| `REACH_END` | 2 | Dubins 直连成功，路径真正到达目标位姿 |
| `NO_PATH` | 3 | OPEN 集为空、节点池耗尽，或没有得到有效路径 |
| `NEAR_END` | 4 | 已到目标附近，但 Dubins 直连失败，仍返回已有搜索段 |
| `ONE_SHOT_FAIL` | 5 | 枚举值被保留，当前 `search()` 没有返回它 |

注意：返回值不是轨迹本身。搜索成功时，函数先通过父指针回溯，把结果保存到内部 `path_nodes_`。

### 4.2 `getKinoTraj()` 的输出：轨迹点序列

管理器在搜索成功后调用：

```cpp
double delta_t_geo = 0.1;
plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(delta_t_geo);
```

输出类型是 `std::vector<Eigen::Vector3d>`，每个元素为 `(x,y,ground_height)`。它由两部分组成：

1. 父指针回溯得到的运动原语搜索段；
2. 若 `is_shot_succ_` 为真，则追加 Dubins 终点直连段。

`delta_t` 是输入输出参数。调用方给出期望时间间隔，函数根据总轨迹时间重新均分后写回实际间隔。输出只有位置点，不直接包含每点的航向角、速度或控制量。管理器对这些点降采样，结合 `getDerivatives()` 给出的首末速度/加速度条件，生成 B 样条控制点。

另有两个调试输出：`getVisitedNodes()` 返回访问节点，`getSearchTree()` 返回运动原语线段，用于可视化搜索过程。

## 5. A* 算法具体使用在哪里

混合 A* 并不是调用了另一个 `Astar` 类，而是在 `KinodynamicAstar2D::search()` 中直接实现了 A* 框架。

### 5.1 OPEN 集

`open_set_` 是优先队列，`NodeComparator` 按 `f_score` 从小到大弹出节点：

```cpp
std::priority_queue<PathNodePtr,
                    std::vector<PathNodePtr>,
                    NodeComparator> open_set_;
```

主循环中的 `open_set_.top()` 对应 A* 每次选取最有希望扩展的节点。

### 5.2 CLOSED 集和状态判重

`expanded_nodes_` 用 `(x_index,y_index,yaw_index)` 查找已发现状态，`node_state` 区分 `IN_OPEN_SET`、`IN_CLOSE_SET` 和 `NOT_EXPAND`。连续状态由 `stateToIndex()` 按位置与航向分辨率量化后，才用于哈希和判重。

### 5.3 评价函数 `f=g+λh`

候选节点的优先级为：

```text
f(n) = g(n) + lambda_heu × h(n)
```

- `g(n)`：从起点到当前节点的累计运动代价，由 `estimateG()` 计算单段路程，并加入转向、转角变化惩罚；
- `h(n)`：由 `estimateHeuristic()` 估计当前节点到目标的剩余代价；
- `lambda_heu`：启发项权重，启动文件当前设置为 `1.05`，因此更准确地说这是加权 A*。它倾向于减少扩展量，但不再严格保证标准 A* 的最优性。

启发函数按场景选择：

- 存在 Guide Path：使用“最近引导点沿路径到终点的剩余长度 + 偏离引导路径的距离惩罚”；
- 不存在 Guide Path，且离目标较远：使用二维欧氏距离；
- 不存在 Guide Path，且进入目标附近：使用 Dubins 距离，把车辆最小转弯半径和目标航向考虑进来。

### 5.4 邻居扩展被车辆运动模型替代

普通栅格 A* 枚举上下左右或八邻域。本实现枚举离散前轮转角和持续时间，然后由 `stateTransit()` 积分运动学自行车模型：

```text
x_dot   = v cos(yaw)
y_dot   = v sin(yaw)
yaw_dot = v tan(steer) / wheel_base
```

这样产生的邻接边天然符合车辆不能横移的非完整约束。每条边还会按 `expand_time_` 分段检查 ESDF 距离，碰撞边不会进入 OPEN 集。

### 5.5 松弛与父指针回溯

如果候选状态尚未发现，则建立新节点并加入 OPEN 集；如果它已经在 OPEN 集中但新路径的 `g_score` 更小，则更新状态、代价和父指针。这就是 A* 的边松弛过程。

达到局部视距、目标附近或成功直连目标后，`retrievePath()` 沿 `parent` 从终止节点回溯到起点，再反转为正向路径。

## 6. 为什么它是“混合”A*，而不是普通 A*

| 对比项 | 普通二维栅格 A* | 本项目混合 A* |
| --- | --- | --- |
| 判重空间 | `(x,y)` 栅格 | `(x,y,yaw)` 栅格，可选时间维 |
| 节点实际状态 | 通常为栅格中心 | 连续的 `x,y,yaw,speed` |
| 邻接边 | 4/8 邻域直线 | 自行车模型生成的曲线运动原语 |
| 车辆转向约束 | 通常不考虑 | 由轴距和转角直接进入状态转移 |
| 目标连接 | 到达目标栅格 | 目标附近尝试 Dubins 解析连接 |
| 输出用途 | 几何折线路径 | B 样条优化的运动学可行初值 |

“离散索引用于搜索管理、连续状态用于运动传播”是该实现最关键的混合特征。

## 7. 主流程逐步解释

1. `reset()` 清空上轮 OPEN 集、哈希表、结果节点和节点状态，但复用预分配节点内存。
2. `setGuidePath()` 保存拓扑引导路径。
3. `search()` 为 Guide Path 计算每一点到终点的剩余折线长度，并建立最近邻 KdTree。
4. 构造起点和目标状态，把起点以 `g=0`、`f=lambda*h` 放入 OPEN 集。
5. 从 OPEN 集弹出 `f` 最小节点，标记为 CLOSED。
6. 检查搜索视距和终点距离；进入直连范围时尝试无碰撞 Dubins 连接。
7. 枚举离散转角、速度方向和持续时间，用自行车模型生成运动原语。
8. 沿运动原语逐小步查询 ESDF；碰撞则丢弃。
9. 把末端连续状态量化为 `(x,y,yaw)` 索引，执行 CLOSED 检查、同体素检查和同父节点剪枝。
10. 计算 `g`、`h`、`f`，插入新节点或松弛已有 OPEN 节点。
11. 成功时沿父指针回溯搜索段；失败时返回 `NO_PATH`。
12. `getKinoTraj()` 根据搜索段总长度设计加速、匀速、减速时序，并按时间输出轨迹点；若直连成功则拼接 Dubins 段。

## 8. 关键参数

| 参数 | 作用 |
| --- | --- |
| `resolution_astar` | `x/y` 状态离散分辨率 |
| `yaw_resolution` | 航向离散分辨率，代码将配置的角度值转为 rad |
| `lambda_heu` | 启发项权重 |
| `horizon` | 局部搜索视距 |
| `max_tau` / `init_max_tau` | 普通/首次运动原语的持续时间尺度 |
| `expand_time` | 运动模型积分与碰撞检测的小步长；未在当前启动文件显式设置时默认 `0.1 s` |
| `max_vel` / `max_acc` | 速度和加速度限制，也参与最终时间参数化 |
| `steering_angle` | 最大前轮转角 |
| `steering_angle_discrete_num` | 最大转角两侧的离散份数 |
| `wheel_base` | 自行车模型轴距，并决定最小转弯半径 |
| `steering_penalty` | 转弯运动原语的代价倍率 |
| `steering_change_penalty` | 改变转角的代价倍率 |
| `obs_dis` | 轨迹点要求的最小障碍距离 |
| `shot_distance` | 开始尝试 Dubins 终点直连的距离 |
| `allocate_num` | 预分配搜索节点数上限 |
| `reverse_enable` | 是否生成倒车输入；当前启动配置为关闭 |

## 9. 阅读代码时需要注意的实现边界

- 当前项目配置关闭倒车，反向路径的启发代价、方向记录和惩罚逻辑仍留有 `TODO`，不能把它视为完整的 Reeds-Shepp 双向混合 A*。
- `dynamic` 时间维接口存在，但当前管理器没有启用；代码主要按静态 ESDF 地图搜索。
- 碰撞检测使用轨迹参考点到障碍物的 ESDF 距离和 `obs_dis_`，没有在此处显式扫描完整矩形车身。车辆长宽参数虽被读取，但没有参与本文件的碰撞判定。
- `w_time_`、`check_num_`、`optimistic_`、`vel_discrete_num_` 等参数在当前二维搜索主流程中没有实际参与代价或扩展。
- Guide Path 启发以及 `lambda_heu > 1` 会明显引导搜索，但该启发不保证满足标准 A* 的可采纳性和一致性，所以不应宣称输出是全局最短路径。
- `getSamples()` 当前会打印“禁用”，实际二维调用使用 `getKinoTraj()` 后再由管理器降采样。

## 10. 仓库中其他 A* 的用途

为避免名称混淆，项目中还存在几类独立的 A*：

- `Astar2D`：由 `TopologyPRM` 持有，主要用于环境变化后历史拓扑路径断裂段的局部重连；它不是本文的车辆混合 A*。
- `Astar`：通用几何/动力学搜索器，只有管理参数 `use_geometric_path` 打开时才创建。
- `GVG.h` 中的 `AstarOnVoronoi`：在 Voronoi/GVG 图结构上进行节点连接搜索。

本文所说的“混合 A*”特指 `KinodynamicAstar2D`，它在 `FastPlannerManager::kinodynamicReplan()` 中承担生成车辆局部初始轨迹的职责。
