你需要在现有 TGH Planner 项目基础上进行 RiskAwareEdge 路径评价模块优化。

重要约束：

1. 不修改 TGH Planner 原有拓扑搜索框架：
   - 不修改 EGVG
   - 不修改 TopoPRM
   - 不修改 B-spline 优化器
   - 不修改 FSM状态机逻辑

2. 所有修改集中在路径评价层：
   RiskAwareEdge / RiskAwarePathSelector 相关模块。

3. 保持 ROS1 Noetic + C++14兼容。

4. 优先保证实时性：
   - 避免引入复杂优化算法
   - 避免增加大量参数
   - 避免动态内存频繁申请

5. 修改代码前：
   - 分析当前代码结构
   - 找出路径评价入口
   - 给出修改计划
   - 不直接大规模重构。

6. 所有新增参数必须：
   - 加入yaml配置
   - 给出默认值
   - 保证旧配置仍可运行。

7. 保留原始接口兼容性。

