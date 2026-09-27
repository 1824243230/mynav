#ifndef _KINODYNAMIC_ASTAR_2D_H
#define _KINODYNAMIC_ASTAR_2D_H

// #include <path_searching/matrix_hash.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include "plan_env/edt_environment.h"
#include <path_searching/kinodynamic_astar.h>
#include "path_searching/dubins.h"
#include <path_searching/jump_point_search.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_types.h>
namespace fast_planner {

/**
 * @brief 面向二维阿克曼车辆的混合 A*（代码沿用 KinodynamicAstar2D 命名）。
 *
 * A* 的 OPEN/CLOSED 集和 f=g+h 搜索运行在离散的 (x,y,yaw) 索引上，
 * 节点之间则由自行车运动学模型生成连续曲线运动原语；这正是“混合”的含义。
 * 搜索结果还可在终点附近拼接 Dubins 曲线，再按时间采样给 B 样条优化器。
 *
 * 调用顺序：setEnvironment() -> setParam() -> init()；每轮规划前 reset()，
 * 可选 setGuidePath()，然后 search()，成功后调用 getKinoTraj()。
 */
class KinodynamicAstar2D {
 private:
  /* ---------- main data structure ---------- */
  vector<PathNodePtr> path_node_pool_;  // 预分配节点池，容量由 allocate_num 参数决定
  int use_node_num_, iter_num_, use_node_num_last_ = 0;
  int use_JPS_times_ = 0; //要连续使用4次JPS引导
  NodeHashTable expanded_nodes_; // 按离散状态索引保存已发现节点，节点自身区分 OPEN/CLOSED
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      open_set_;  // A* OPEN 集，f_score 最小的节点优先弹出
  std::vector<PathNodePtr> path_nodes_; // 父指针回溯并反转后的节点序列，包含起点

  // ---------- JPS path ---- ---------//
  unique_ptr<JumpPointSearch> jps_path_finder_;
  bool useJPS_, succJPS_ = false;
  bool use_kino_replan_ = true; // true表示用kino replanning，false表示用cmu_planner
  std::vector<double> jps_path_distance_;
  pcl::KdTreeFLANN<pcl::PointXY> JPSPathKdTree;

  // ---------- Topo path -------------//
  vector<Eigen::Vector3d> topo_path_;     // 上层传入的稠密拓扑引导路径
  std::vector<double> topo_path_distance_; // 每个引导点沿折线到终点的剩余距离
  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_, start_pt_;
  // shared_ptr<SDFMap> sdf_map;
  EDTEnvironment::Ptr edt_environment_;
  bool is_shot_succ_ = false;
  double total_t_;            //总的轨迹时间
  int case_id_;
  double t1_, t2_, t3_, v_p_;
  double shot_len_, expand_len_, total_len_;
  bool has_path_ = false;
  DubinsPath::DubinsPath path_;
  double one_shot_vel_;      //one shot时使用的速度
  vector<double> steering_angle_rads_;
  double ground_height_;
  double start_yaw_, end_yaw_;
  /* ---------- parameter ---------- */
  /* search */
  // max_tau_ = 0.6， init_max_tau_ = 0.8
  double max_tau_, init_max_tau_;
  double max_vel_, max_acc_;
  double w_time_, horizon_, lambda_heu_;
  int allocate_num_, check_num_;
  double tie_breaker_;
  bool optimistic_;
  double obs_dis_;

  double steering_angle_;
  int steering_angle_discrete_num_;
  int vel_discrete_num_;      
  double wheel_base_;         //轴距，即前后轮的距离
  double steering_radius_;
  double steering_penalty_, steering_change_penalty_, reversing_penalty_;
  double vehicle_length_, vehicle_width_, vehicle_rear_dis_;
  double steering_radian_;
  bool reverse_enable_;
  double shot_distance_;
  double expand_time_;        //扩展节点时，每一个中间点的时间间隔，设为0.1s。越小轨迹就越准确。这里，扩展时的、getKinoTraj和getSample的时间间隔都必须一样才能保证轨迹时一样的
  int mid_states_size_;
  bool show_search_tree_ = false; //现在这个不能关闭了，因为涉及到getKinoTraj()
  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;

  double yaw_resolution_, inv_yaw_resolution_;
  /* helper */
  // 连续 (x,y,yaw) 量化为哈希键，这是混合 A* 的离散搜索部分。
  Eigen::Vector3i stateToIndex(Eigen::Vector3d state);
  Eigen::Vector2i stateToIndex2D(Eigen::Vector2d state);
  Eigen::Vector2d indexToState2D(Eigen::Vector2i index);
  int timeToIndex(double time);
  void retrievePath(PathNodePtr end_node); // 沿 parent 回溯搜索段

  /* shot trajectory */
  // 检查当前状态到目标状态的 Dubins 解析连接是否无碰撞。
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                       double time_to_goal);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                           double& optimal_time); // A* 的 h：引导路径代价或欧氏/Dubins 距离
  // A* 的单段 g：路程，并对转向及转向变化施加惩罚。
  double estimateG(const Eigen::Matrix<double, 6, 1>& state0, const int & steering0,
                  const Eigen::Matrix<double, 6, 1>& state1, const int & steering1,
                  double ts) const;
  /* state propagation */
  // 自行车模型积分，控制量 um=[纵向速度, 前轮转角]，tau 为持续时间。
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0, 
                    Eigen::Matrix<double, 6, 1>& state1,
                    Eigen::Vector2d um, double tau);
  int calVelOnShotTraj(const double& v0);
  double getDist(const double& t, const double & v0);
  double calScaleFactor(const double& t, const double & v0, const double& curv_tmp);
 public:
  KinodynamicAstar2D(){};
  ~KinodynamicAstar2D();

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4, ONE_SHOT_FAIL = 5 };

  /* main API */
  void setParam(ros::NodeHandle& nh);
  void init();
  void reset();
  /**
   * @brief 执行一次混合 A* 搜索。
   * @param start_pt 起点世界坐标 (x,y,z)，搜索使用 x/y，z 作为输出高度。
   * @param start_vel 起点速度向量，模长作为初始标量速度。
   * @param start_acc 起点加速度，供后续 B 样条边界条件使用。
   * @param start_yaw 起点航向角，单位 rad。
   * @param end_pt 目标世界坐标 (x,y,z)，搜索使用 x/y。
   * @param end_vel 目标速度向量，模长写入目标状态。
   * @param end_yaw 目标航向角，单位 rad。
   * @param init true 时第一段保持当前速度并直行，以增强轨迹连续性。
   * @param dynamic true 时将离散时间加入节点键；当前车辆调用采用默认 false。
   * @param time_start 动态搜索起始时刻，仅 dynamic=true 时使用。
   * @param gen_search 预留参数，当前实现未使用。
   * @return REACH_END、REACH_HORIZON、NEAR_END 或 NO_PATH。
   */
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
             Eigen::Vector3d start_acc, double start_yaw,
             Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, double end_yaw, 
             bool init, bool dynamic = false,
             double time_start = -1.0, bool gen_search = 1);

  void setEnvironment(const EDTEnvironment::Ptr& env);

  /**
   * @brief 将搜索运动原语和可选 Dubins 直连段按时间采样为位置序列。
   * @param[in,out] delta_t 期望采样周期；函数会调整为可均分总时长的周期。
   * @return 世界坐标轨迹点 (x,y,ground_height)，供可视化及 B 样条参数化。
   */
  std::vector<Eigen::Vector3d> getKinoTraj(double& delta_t);

  void getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                  vector<Eigen::Vector3d>& start_end_derivatives);
  void getDerivatives(vector<Eigen::Vector3d>& start_end_derivatives);
  std::vector<PathNodePtr> getVisitedNodes();
  std::vector<Eigen::Vector3d> getJpsPath() 
  {
    if(useJPS_) return jps_path_finder_->getJpsPath();
    else return std::vector<Eigen::Vector3d>();
  }
  bool showSearchTree()
  {
    return show_search_tree_;
  }
  void setGuidePath(const vector<Eigen::Vector3d>& topo_path)
  {
    // 引导路径不是硬约束，只通过 estimateHeuristic() 改变节点扩展优先级。
    this->topo_path_ = topo_path;
  }
  vector<Eigen::Vector4d> getSearchTree();
  double getSteerRadius() {return steering_radius_;}
  typedef shared_ptr<KinodynamicAstar2D> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif
