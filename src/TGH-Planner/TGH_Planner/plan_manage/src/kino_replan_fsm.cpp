/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/




#include <plan_manage/kino_replan_fsm.h>

namespace fast_planner {

void KinoReplanFSM::init(ros::NodeHandle& nh) {
  trigger_ = false;
  have_target_ = false;
  have_odom_ = false;
  planning_busy_ = false;
  replan_pending_ = false;
  active_trajectory_unsafe_ = false;
  topology_reset_pending_ = false;
  last_plan_time_ = 0.0;
  last_planning_attempt_wall_time_ = ros::WallTime(0);
  current_wp_ = 0;
  exec_state_ = FSM_EXEC_STATE::INIT;
  odom_pos_.setZero();
  odom_vel_.setZero();
  odom_orient_.setIdentity();
  start_pt_.setZero();
  start_vel_.setZero();
  start_acc_.setZero();
  start_yaw_.setZero();
  end_pt_.setZero();
  end_vel_.setZero();
  end_yaw_.setZero();

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);      //
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);//
  nh.param("fsm/planning_retry_interval", planning_retry_interval_, 0.5);
  planning_retry_interval_ = std::max(0.05, planning_retry_interval_);

  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  waypoint_num_ = std::max(0, std::min(50, waypoint_num_));
  nh.param("fsm/B_Spline_LocalPlanner", use_kino_replan_, true);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }


  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  exec_timer_   = nh.createTimer(ros::Duration(use_kino_replan_ ? 0.01 : 0.1), &KinoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);
  odom_record_timer_ = nh.createTimer(ros::Duration(0.1), &KinoReplanFSM::odomRecordCallback, this);
  topo_update_timer_ = nh.createTimer(ros::Duration(0.1), &KinoReplanFSM::TopoContainerUpdate, this);

  waypoint_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);

  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
  reset_srv_ = nh.advertiseService("/planning/reset_env", &fast_planner::KinoReplanFSM::reset_env, this);
  waypoint_pub_ = nh.advertise<geometry_msgs::PointStamped>("/path_execution_node/look_ahead_goal", 10);
}

void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses.empty()) {
    ROS_WARN("Ignore empty waypoint message.");
    return;
  }
  if (msg->poses[0].pose.position.z < -0.1) return;

  cout << "Triggered!" << endl;
  trigger_ = true;

  // 将终点存起来
  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    end_yaw_ << tf::getYaw(msg->poses[0].pose.orientation), 0.0, 0.0;

  } 
  //相当于这里，即使设置了一系列waypoints，但是终点的切换还是手动完成的
  else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    if (waypoint_num_ == 0) {
      ROS_ERROR("Preset target requested without configured waypoints.");
      return;
    }
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }
  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  if (planning_busy_) {
    replan_pending_ = true;
    topology_reset_pending_ = true;
    ROS_INFO("Planner busy; coalesced waypoint replan request.");
    return;
  }

  planner_manager_->resetTopoPathContainer();
  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");//TRIG是指指定了终点
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[6] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "REPLAN_NEW" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[6] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "REPLAN_NEW" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

bool KinoReplanFSM::planningRetryReady() const {
  if (last_planning_attempt_wall_time_.toSec() <= 0.0) return true;
  return (ros::WallTime::now() - last_planning_attempt_wall_time_).toSec() >=
         planning_retry_interval_;
}

bool KinoReplanFSM::pendingReplanStillRequired() const {
  if (!have_target_ || !have_odom_) return false;
  if (active_trajectory_unsafe_ || !planner_manager_->hasActivePlan()) {
    return true;
  }

  NonUniformBspline active_position =
      planner_manager_->local_data_.position_traj_;
  const double duration = active_position.getTimeSum();
  if (duration <= 0.0) return true;
  const Eigen::Vector3d active_end =
      active_position.evaluateDeBoorT(duration);
  return (active_end - end_pt_).norm() >
         std::max(0.1, no_replan_thresh_);
}

void KinoReplanFSM::requestReplan(bool emergency_stop,
                                  const string& source) {
  if (!have_target_) return;

  if (emergency_stop) {
    const bool first_invalidation = !active_trajectory_unsafe_;
    active_trajectory_unsafe_ = true;
    if (first_invalidation && planner_manager_->hasActivePlan()) {
      // This is the only normal producer of /planning/replan in Kino FSM.
      // traj_server may truncate the old trajectory because it is confirmed
      // unsafe, not merely because a replacement is being attempted.
      replan_pub_.publish(std_msgs::Empty());
      ROS_ERROR("Emergency stop: invalidating unsafe ActivePlan.");
    }
  }

  if (planning_busy_) {
    replan_pending_ = true;
    ROS_INFO_STREAM("Planner busy; coalesced replan request from " << source);
    return;
  }

  const FSM_EXEC_STATE next_state =
      (!planner_manager_->hasActivePlan() || active_trajectory_unsafe_)
          ? GEN_NEW_TRAJ
          : REPLAN_TRAJ;
  if (exec_state_ != next_state) changeFSMExecState(next_state, source);
}

bool KinoReplanFSM::tryPlanningAttempt(bool& success) {
  if (planning_busy_) {
    replan_pending_ = true;
    return false;
  }
  if (!planningRetryReady()) return false;

  planning_busy_ = true;
  last_planning_attempt_wall_time_ = ros::WallTime::now();
  success = callKinodynamicReplan();
  planning_busy_ = false;
  return true;
}

void KinoReplanFSM::finishPlanningAttempt(bool success,
                                          bool initial_attempt) {
  if (success) {
    active_trajectory_unsafe_ = false;
    last_plan_time_ = 0.0;
    changeFSMExecState(EXEC_TRAJ, "FSM");
  } else if (initial_attempt || active_trajectory_unsafe_ ||
             !planner_manager_->hasActivePlan()) {
    changeFSMExecState(GEN_NEW_TRAJ, "FSM");
  } else {
    ROS_WARN("CandidatePlan failed; continuing the current ActivePlan.");
    changeFSMExecState(EXEC_TRAJ, "FSM");
  }

  if (!replan_pending_) return;

  replan_pending_ = false;
  const bool retry_required = pendingReplanStillRequired();
  if (retry_required) {
    if (topology_reset_pending_) planner_manager_->resetTopoPathContainer();
    topology_reset_pending_ = false;
    const FSM_EXEC_STATE next_state =
        (!planner_manager_->hasActivePlan() || active_trajectory_unsafe_)
            ? GEN_NEW_TRAJ
            : REPLAN_TRAJ;
    changeFSMExecState(next_state, "PENDING");
    ROS_INFO("Scheduling one coalesced pending replan attempt.");
  } else {
    topology_reset_pending_ = false;
    ROS_INFO("Dropping stale pending replan request.");
  }
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      if (planning_busy_ || !planningRetryReady()) return;
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = false;
      if (!tryPlanningAttempt(success)) return;
      finishPlanningAttempt(success, true);
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();
      t_cur                   = min(info->duration_, t_cur);

      // 为什么要计算这个pos？这是当前时间的轨迹的位置，也就是期望的？为什么不向前计算一点？
      // 因为轨迹就是带时间的，只要每个周期都执行好当前的就可以了。
      // 注意看，这个pos是轨迹上随着时间取的，而不是真正的位置
      // 所以在仿真中，即使机器人没动，这个pos也是一直在往前走。触发重规划
      if (!use_kino_replan_) 
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");//如果是cmu_planner，就直接周期性地重规划
        return;
      }
      
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
        // cout << "near end" << endl;
        return;

      } 
      else if((t_cur - last_plan_time_ > 0.8))
      {
        last_plan_time_ = t_cur;
        changeFSMExecState(REPLAN_TRAJ, "FSM");//周期性地在进行重规划
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_) {
        // cout << "----------near start------------" << endl;
        return;

      } else {
        // cout << "-----------:" << (info->start_pos_ - pos).norm() << ", check?-----------------" << std::endl; 
        last_plan_time_ = t_cur;
        changeFSMExecState(REPLAN_TRAJ, "FSM");//周期性地在进行重规划
      }
      break;
    }

    case REPLAN_TRAJ: {
      if (planning_busy_ || !planningRetryReady()) return;
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();
      t_cur = std::max(0.0, std::min(info->duration_, t_cur));

      if(planner_manager_->only2D())
      {
        start_pt_  = odom_pos_;
        start_vel_ = odom_vel_;
        if(use_kino_replan_) start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
        else start_acc_ = Eigen::Vector3d::Zero();

        Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
        start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
        start_yaw_(1) = start_yaw_(2) = 0.0;

      }
      else
      {
        start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
        start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
        start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

        start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
        start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
        start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];        
      }

      bool success = false;
      if (!tryPlanningAttempt(success)) return;
      finishPlanningAttempt(success, false);
      break;
    }
    case REPLAN_NEW:
      changeFSMExecState(REPLAN_TRAJ, "FSM");
      break;
  }
}

void KinoReplanFSM::odomRecordCallback(const ros::TimerEvent& e)
{
  if(!have_odom_ || planning_busy_) return;
  // 这个只能是-0.5，和topo路径搜索时的z轴一致
  Eigen::Vector3d odom_pos(odom_pos_.x(), odom_pos_.y(), -0.5);
  if(start_change_.empty())
    start_change_.emplace_back(odom_pos);
  else if((start_change_.back() - odom_pos).squaredNorm() > 0.01)
    start_change_.emplace_back(odom_pos);
}

void KinoReplanFSM::TopoContainerUpdate(const ros::TimerEvent& e)
{
  if (planning_busy_) return;
  // ROS_WARN("Update TopoPath Container!");
  planner_manager_->topoUpdate(start_change_);
  start_change_.resize(0);
}

//当前的轨迹和新观察到的障碍物碰撞了
void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;
    auto only2D = planner_manager_->only2D();
    auto obj_predictor = planner_manager_->obj_predictor_;
    // DYNAMIC
    if(planner_manager_->pp_.dynamic_)
    {
      edt_env->setObjPrediction(obj_predictor->getPredictionTraj());
      edt_env->setObjScale(obj_predictor->getObjScale());
    }
    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_, only2D) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0, only2D);

    //当前的end_pt_！发生了碰撞，那么就在终点附近重新找一个最好的无碰撞的end_pt_
    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      const double    dr = 0.5, dtheta = 30, dz = 0.3;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_, only2D) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0, only2D);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        requestReplan(false, "SAFETY_GOAL");

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        requestReplan(false, "SAFETY_GOAL");
      }
    }
  }

  /* ---------- check trajectory ---------- */
  // 如果是轨迹发生了碰撞，那么就立即重新规划一条轨迹
  // 如果use_kino_replan_为false，说明用的是cmu_planner，这个planner不检查轨迹碰撞
  if ((exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ ||
       (planning_busy_ && planner_manager_->hasActivePlan())) &&
      use_kino_replan_) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      requestReplan(true, "SAFETY_TRAJECTORY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {

  const ros::WallTime replanning_begin = ros::WallTime::now();

  planner_manager_->TopoPathReplan(start_pt_, end_pt_, start_yaw_, start_change_);
  if (!use_kino_replan_)
  {
    planner_manager_->discardCandidatePlan();
    ROS_WARN("CandidatePlan was not generated because kinodynamic replanning is disabled.");
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGuidePath(plan_data->topo_guide_path_, 0.075, Eigen::Vector4d(0.5, 0.5, 0.0, 1.0));
    visualization_->drawTopoGraph(plan_data->topo_graph_, 0.2, 0.05, Eigen::Vector4d(1.0, 0, 0.0, 1.0), 
                                  Eigen::Vector4d(0.0, 1, 0.0, 1.0), Eigen::Vector4d(0.0, 1, 0.0, 1.0));
    visualization_->drawTopoPathsPhase1(plan_data->topo_filtered_paths_, 0.07);
    visualization_->drawTopoPathsPhase2(plan_data->topo_select_paths_, 0.15);

    visualization_->drawTopoVoronoiPaths(plan_data->voronoi_paths_, 0.2);

    // // 将waypoint发出去
    // if (plan_data->waypoint_.z() != -1000)
    // {
    //   geometry_msgs::PointStamped wp_msg;
    //   wp_msg.header.stamp = ros::Time::now();
    //   wp_msg.header.frame_id = "world";
    //   wp_msg.point.x = plan_data->waypoint_(0);
    //   wp_msg.point.y = plan_data->waypoint_(1);
    //   wp_msg.point.z = plan_data->waypoint_(2);
    //   waypoint_pub_.publish(wp_msg);    
    // }

    ROS_DEBUG_STREAM("[IncrementalTopo] total_replanning_time="
                     << (ros::WallTime::now() - replanning_begin).toSec() * 1000.0 << "ms");
    return false;
  }


  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, start_yaw_, end_yaw_, start_change_);
  start_change_.resize(0);
  if (plan_success &&
      planner_manager_->commitCandidatePlan(start_yaw_, end_yaw_)) {

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();
    // std::cout << "position crt_pt size: " << bspline.pos_pts.size() << ", yaw crt_pt size: " << yaw_pts.rows() << std::endl;
    // std::cout << "position knots: " << knots.transpose() << std::endl;
    bspline_pub_.publish(bspline);
    ROS_WARN("Pub New Traj!!");
    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.09, Eigen::Vector4d(1, 0.8, 0.2, 0.9));
    visualization_->drawBspline(info->position_traj_, 0.08, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));
    visualization_->drawGuidePath(plan_data->topo_guide_path_, 0.075, Eigen::Vector4d(0.5, 0.5, 0.0, 1.0));

    visualization_->drawYawTraj(info->position_traj_, info->yaw_traj_, plan_data->dt_yaw_);
    visualization_->drawVisitedNodes(plan_data->visited_nodes);
    visualization_->drawSearchTree(plan_data->search_tree, -0.5, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
    // // //画出时间优化前的B样条曲线，看看
    visualization_->drawBspline(info->position_traj_tmp_, 0.08, Eigen::Vector4d(0.0, 0, 0.0, 0.7), true, 0.2,
                                Eigen::Vector4d(0, 0, 0, 0.7), 1, 1, 1);
    visualization_->drawTopoGraph(plan_data->topo_graph_, 0.2, 0.05, Eigen::Vector4d(1.0, 0, 0.0, 1.0), 
                                  Eigen::Vector4d(0.0, 1, 0.0, 1.0), Eigen::Vector4d(0.0, 1, 0.0, 1.0));
    visualization_->drawTopoPathsPhase1(plan_data->topo_filtered_paths_, 0.07);
    visualization_->drawTopoPathsPhase2(plan_data->topo_select_paths_, 0.15);

    visualization_->drawTopoVoronoiPaths(plan_data->voronoi_paths_, 0.2);

    // visualization_->drawTopoSampleArea(plan_data->topo_sample_area_, 0.05, Eigen::Vector4d(0.5, 0.5, 0.5, 0.5));
    visualization_->drawPerceptionInfo(plan_data->block_pts_, 0.3);

    ROS_DEBUG_STREAM("[IncrementalTopo] total_replanning_time="
                     << (ros::WallTime::now() - replanning_begin).toSec() * 1000.0 << "ms");
    return true;

  } else {
    planner_manager_->rejectCandidatePlan();
    cout << "generate new traj fail." << endl;
    ROS_DEBUG_STREAM("[IncrementalTopo] total_replanning_time="
                     << (ros::WallTime::now() - replanning_begin).toSec() * 1000.0 << "ms");
    return false;
  }
}

bool KinoReplanFSM::reset_env(common_srvs::reset_env::Request &req, common_srvs::reset_env::Response &res)
{
  if (planning_busy_) {
    ROS_WARN("Reject reset_env while planner is busy.");
    replan_pending_ = true;
    res.success = false;
    return true;
  }
  ROS_WARN("Receive Service Call!");
  this->planner_manager_->edt_environment_->sdf_map_->resetBuffer();
  std_msgs::Empty emt;
  this->new_pub_.publish(emt);
  res.success = true;
  return true;
}


// KinoReplanFSM::
}  // namespace fast_planner
