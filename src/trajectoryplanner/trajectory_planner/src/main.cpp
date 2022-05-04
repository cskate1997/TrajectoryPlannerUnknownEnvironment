#include <sensor_msgs/point_cloud_conversion.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <snapstack_msgs/State.h>
#include <snapstack_msgs/Goal.h>
#include <custom_msgs/Mode.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "custom_types.hpp"
#include "utils.hpp"
#include <pcl/kdtree/kdtree.h>
#include <Eigen/StdVector>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <stdlib.h>
#include "timer.hpp"
#include "termcolor.hpp"
#include "solverGurobi.hpp"
#include <assert.h>
#include <mutex>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include "termcolor.hpp"
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>
#include "read_map.hpp"
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <std_msgs/Int8.h>

enum TurtlebotStatus
{
  Searching = 0,
  in_motion = 1,
  goal_in_map = 2,
  goal_reached = 3
};

enum CurrentStatus
{
  intial_plan = 0,
  replanning = 1,
  Task_accomplished = 2
};

using namespace JPS;
using namespace termcolor;
typedef Timer MyTimer;

class JPS_Wrapper
{
public:
  JPS_Wrapper() 
  { 
    map_pointer = std::make_shared<JPS::VoxelMapUtil>();
    planner_pointer = std::unique_ptr<JPSPlanner3D>(new JPSPlanner3D(false));
  }

  vec_Vec3f occupied_map;   
  vec_Vec3f unoccupied_map; 
  std::shared_ptr<JPS::VoxelMapUtil> map_pointer;
  std::unique_ptr<JPSPlanner3D> planner_pointer;
  std::mutex mute_jps_map;

  void Z_limit_Setter(double z_ground, double z_max)
  {
    z_ground_ = z_ground;
    z_max_ = z_max;
  }
  void setFactorJPS(double factor_jps)
  { 
    factor_jps_ = factor_jps;
  }

  void setInflationJPS(double inflation_jps)
  {
    inflation_jps_ = inflation_jps;
  }

  void Resolution_Member(double resolution)
  {
    resolution_ = resolution;
  }

  void setVisual(bool visual)
  {
    visual_ = visual;
  }

  void Approx_Bot_Radius(double radius)
  {
    bot_radius_ = radius;
  }

  void updateJPSMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr, Eigen::Vector3d& center) 
  {
    Vec3f center_map = center;
    mute_jps_map.lock();
    map_pointer->readMap(pclptr, x_, y_, z_, factor_jps_ * resolution_, center_map, z_ground_, z_max_, inflation_jps_);
    mute_jps_map.unlock();
  }

  vec_Vecf<3> JPS_planner(Vec3f& start_point, Vec3f& goal_point, bool* solved, int i) 
  {
    Eigen::Vector3d start(start_point(0), start_point(1), std::max(start_point(2), 0.0));
    Eigen::Vector3d goal(goal_point(0), goal_point(1), std::max(goal_point(2), 0.0));
    Vec3f originalStart = start;
    pcl::PointXYZ pcl_start = eigenPoint2pclPoint(start);
    pcl::PointXYZ pcl_goal = eigenPoint2pclPoint(goal);
    mute_jps_map.lock();
    const Veci<3> start_pt = map_pointer->floatToInt(start);
    const Veci<3> goal_pt = map_pointer->floatToInt(goal);
    map_pointer->setFreeVoxelAndSurroundings(start_pt, inflation_jps_);
    map_pointer->setFreeVoxelAndSurroundings(goal_pt, inflation_jps_);
    planner_pointer->setMapUtil(map_pointer);
    bool path_jps = planner_pointer->plan(start, goal, 1, true);  
    vec_Vecf<3> path;
    path.clear();
    if (path_jps == true)
    {
      path = planner_pointer->getPath();
      if (path.size() > 1)
      {
        path[0] = start;
        path[path.size() - 1] = goal;
      }
      else
      { 
        vec_Vecf<3> temp;
        temp.push_back(start);
        temp.push_back(goal);
        path = temp;
      }
    }
    else
    {
      std::cout << "JPS didn't find a solution from" << start.transpose() << " to " << goal.transpose() << std::endl;
    }
    mute_jps_map.unlock();

    *solved = path_jps;
    return path;
  }

  void Setter_Num_Cells(int x, int y, int z) 
  {
    x_ = x;
    y_ = y;
    z_ = z;
  }

  // Convex Decomposition
  void get_Safe_Zone(vec_Vecf<3>& path, int search_type, std::vector<LinearConstraint3D>& set_contraints, vec_E<Polyhedron<3>>& Output_Polyhedron)
  {
    if (search_type == UNKOWN_AND_OCCUPIED_SPACE)
      {
        Decomp_Obj.set_obs(unoccupied_map);
      }
    else
      {
        Decomp_Obj.set_obs(occupied_map);
      }

    Decomp_Obj.set_local_bbox(Vec3f(2, 2, 1));                                               
    Decomp_Obj.set_inflate_distance(bot_radius_);  
    Decomp_Obj.dilate(path);                         
    auto Polyhedrons = Decomp_Obj.get_polyhedrons();

    set_contraints.clear();

    for (size_t i = 0; i < path.size() - 1; i++)
      {
        const auto point_inside = (path[i] + path[i + 1]) / 2;
        LinearConstraint3D constraint_obj(point_inside, Polyhedrons[i].hyperplanes());
        constraint_obj.A_.conservativeResize(constraint_obj.A_.rows() + 1, constraint_obj.A_.cols());
        constraint_obj.A_.row(constraint_obj.A_.rows() - 1) = -Eigen::Vector3d::UnitZ();
        constraint_obj.b_.conservativeResize(constraint_obj.b_.rows() + 1, constraint_obj.b_.cols());
        constraint_obj.b_[constraint_obj.b_.rows() - 1] = -z_ground_;

        set_contraints.push_back(constraint_obj);
      }
    Output_Polyhedron = Decomp_Obj.get_polyhedrons();
  }

private:
  double factor_jps_, resolution_, inflation_jps_, z_ground_, z_max_, bot_radius_;
  int x_, y_, z_;
  bool visual_;
  EllipsoidDecomp3D Decomp_Obj;
};

class Local_Planner
{
public:
  Local_Planner(parameters parameters_) : bot_parameter_(parameters_) 
  {
    turtlebot_state_ == TurtlebotStatus::Searching;
    temp_goal_inside_map.pos << 0, 0, 0;
    Final_goal_.pos << 0, 0, 0;
    mutex_obj_1.lock();
    stateA_.setZero();
    mutex_obj_1.unlock();
    jps_wrapper_obj.Setter_Num_Cells((int)bot_parameter_.wdx / bot_parameter_.res, (int)bot_parameter_.wdy / bot_parameter_.res, (int)bot_parameter_.wdz / bot_parameter_.res);
    jps_wrapper_obj.setFactorJPS(bot_parameter_.factor_jps);
    jps_wrapper_obj.Resolution_Member(bot_parameter_.res);
    jps_wrapper_obj.setInflationJPS(bot_parameter_.inflation_jps);
    jps_wrapper_obj.Z_limit_Setter(bot_parameter_.z_ground, bot_parameter_.z_max);
    jps_wrapper_obj.Approx_Bot_Radius(bot_parameter_.turtlebot_radius);
    double max_values[3] = { bot_parameter_.v_max, bot_parameter_.a_max, bot_parameter_.j_max };

// Feed parameters to Gurobi
    // Setup of whole_traj_gurobi
    whole_traj_gurobi.setN(bot_parameter_.N_whole);
    whole_traj_gurobi.createVars();
    whole_traj_gurobi.setDC(bot_parameter_.dc);
    whole_traj_gurobi.setBounds(max_values);
    whole_traj_gurobi.setForceFinalConstraint(true);
    whole_traj_gurobi.setFactorInitialAndFinalAndIncrement(1, 10, bot_parameter_.increment_whole);
    whole_traj_gurobi.setVerbose(bot_parameter_.gurobi_verbose);
    whole_traj_gurobi.setThreads(bot_parameter_.gurobi_threads);
    whole_traj_gurobi.setWMax(bot_parameter_.w_max);
// Feed parameters to Gurobi
    // Setup of safe_traj_gurobi
    safe_traj_gurobi.setN(bot_parameter_.N_safe);
    safe_traj_gurobi.createVars();
    safe_traj_gurobi.setDC(bot_parameter_.dc);
    safe_traj_gurobi.setBounds(max_values);
    safe_traj_gurobi.setForceFinalConstraint(false);
    safe_traj_gurobi.setFactorInitialAndFinalAndIncrement(1, 10, bot_parameter_.increment_safe);
    safe_traj_gurobi.setVerbose(bot_parameter_.gurobi_verbose);
    safe_traj_gurobi.setThreads(bot_parameter_.gurobi_threads);
    safe_traj_gurobi.setWMax(bot_parameter_.w_max);

    pclptr_unk_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pclptr_map_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    changeBotStatus(TurtlebotStatus::goal_reached);
    hard_reset();
  }
 
  void setTerminalGoal(state& term_goal) 
  {
    mtx_G_term.lock();
    mtx_G.lock();
    mtx_state.lock();
    mtx_planner_status_.lock();

    Final_goal_.pos = term_goal.pos;
    Eigen::Vector3d temp = state_.pos;
    temp_goal_inside_map.pos = projectPointToBox(temp, Final_goal_.pos, bot_parameter_.wdx, bot_parameter_.wdy, bot_parameter_.wdz);
    if (turtlebot_state_ == TurtlebotStatus::goal_reached)
    {
      changeBotStatus(TurtlebotStatus::Searching);
    }
    terminal_goal_initialized_ = true;

    mtx_state.unlock();
    mtx_G.unlock();
    mtx_G_term.unlock();
    mtx_planner_status_.unlock();
  }
  
  void hard_reset() 
  {
    planner_initialized_ = false;
    state_initialized_ = false;
    kdtree_map_initialized_ = false;
    kdtree_unk_initialized_ = false;
    terminal_goal_initialized_ = false;
  }
  
  void updateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map, pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk) 
  {
    mtx_map.lock();
    mtx_unk.lock();
    pclptr_map_ = pclptr_map;
    pclptr_unk_ = pclptr_unk;
    jps_wrapper_obj.updateJPSMap(pclptr_map_, state_.pos);  
    if (pclptr_map_->width != 0 && pclptr_map_->height != 0) 
    {
      kdtree_map_.setInputCloud(pclptr_map_);
      kdtree_map_initialized_ = 1;
      jps_wrapper_obj.occupied_map = pclptr_to_vec(pclptr_map_);
    }
    else 
    {
      std::cout << "Occupancy Grid received is empty, maybe map is too small?" << std::endl;
    }

    if (pclptr_unk_->points.size() == 0) 
    {
      std::cout << "Unkown cloud has 0 points" << std::endl;
      return;
    }
    else 
    {
      kdtree_unk_.setInputCloud(pclptr_unk_);
      kdtree_unk_initialized_ = 1;
      jps_wrapper_obj.unoccupied_map = pclptr_to_vec(pclptr_unk_); 
      jps_wrapper_obj.unoccupied_map.insert(jps_wrapper_obj.unoccupied_map.end(), jps_wrapper_obj.occupied_map.begin(),jps_wrapper_obj.occupied_map.end()); 
    }
    mtx_map.unlock();
    mtx_unk.unlock();
  }
  
  void updateState(state data) 
  {
    state_ = data;

    if (state_initialized_ == false) {
      state temp;
      temp.pos = data.pos;
      temp.theta = data.theta;
      plan_.push_back(temp);
    }
    state_initialized_ = true;
  }

  void replan(vec_Vecf<3>& Safe_traj_JPS, vec_Vecf<3>& whole_traj_JPS, vec_E<Polyhedron<3>>& safe_op_polyhedron, vec_E<Polyhedron<3>>& whole_op_polyhedron, std::vector<state>& X_safe_out, std::vector<state>& X_whole_out) {
                MyTimer replanCB_t(true);
    if (initializedAllExceptPlanner() == false)
    {
      return;
    }

    whole_traj_gurobi.ResetToNormalState();
    safe_traj_gurobi.ResetToNormalState();

    mtx_state.lock();
    mtx_G.lock();
    mtx_G_term.lock();

    state state_local = state_;
    state GOAL_pos;
    GOAL_pos.pos = projectPointToBox(state_local.pos, Final_goal_.pos, bot_parameter_.wdx, bot_parameter_.wdy, bot_parameter_.wdz);
    state G_term = Final_goal_; 

    mtx_G.unlock();
    mtx_G_term.unlock();
    mtx_state.unlock();

    // Check if we have reached the goal
    double dist_to_goal = (G_term.pos - state_local.pos).norm();
    if (dist_to_goal < bot_parameter_.goal_radius)
    {
      changeBotStatus(TurtlebotStatus::goal_reached);
    }

    // Don't plan if robot is not traveling
    if (turtlebot_state_ == TurtlebotStatus::goal_reached || (turtlebot_state_ == TurtlebotStatus::Searching))
    {
      std::cout << "No replanning needed because" << std::endl;
      print_status();
      return;
    }

    state A;
    int k_safe, k_end_whole;
    k_end_whole = std::max((int)plan_.size() - deltaTime_, 0);
    A = plan_[plan_.size() - 1 - k_end_whole];

    bool solvedjps = false;
    MyTimer timer_jps(true);

    vec_Vecf<3> JPS_k_obj = jps_wrapper_obj.JPS_planner(A.pos, GOAL_pos.pos, &solvedjps, 1);

    if (solvedjps == false)
    {
      std::cout << bold << red << "JPS didn't find a solution" << std::endl;
      return;
    }

    double ellipse_radius = std::min((dist_to_goal - 0.001), bot_parameter_.Ra);  // radius of the sphere S
    
    bool Any_Points_OutSide_S;
    int last_index_inside_sphere;  // last index inside the sphere of JPS_k_obj
    state E;
    E.pos = getFirstIntersectionWithSphere(JPS_k_obj, ellipse_radius, JPS_k_obj[0], &last_index_inside_sphere, &Any_Points_OutSide_S);
    vec_Vecf<3> pts_in_jps(JPS_k_obj.begin(), JPS_k_obj.begin() + last_index_inside_sphere + 1);
    if (Any_Points_OutSide_S == false)
    {
      pts_in_jps.push_back(E.pos);
    }
    
    createMoreVertexes(pts_in_jps, bot_parameter_.dist_max_vertexes);

    if (bot_parameter_.use_faster == true)
    {
      vec_Vecf<3> whole_jps_traj = pts_in_jps;
      deleteVertexes(whole_jps_traj, bot_parameter_.max_poly_whole);
      E.pos = whole_jps_traj[whole_jps_traj.size() - 1];
      MyTimer time_for_decomposition(true);
      jps_wrapper_obj.get_Safe_Zone(whole_jps_traj, OCCUPIED_SPACE, l_constraints_whole_, whole_op_polyhedron);
      bool isGinside_whole = l_constraints_whole_[l_constraints_whole_.size() - 1].inside(GOAL_pos.pos);
      E.pos = (isGinside_whole == true) ? GOAL_pos.pos : E.pos;
      whole_traj_gurobi.setX0(A);
      whole_traj_gurobi.setXf(E);
      whole_traj_gurobi.setPolytopes(l_constraints_whole_);
      // Solve with Gurobi
      MyTimer time_whole_gurobi(true);
      bool found_whole = whole_traj_gurobi.genNewTraj();
      whole_traj_JPS = whole_jps_traj;
      if (found_whole == false)
      {
        std::cout <<"No solution found for the whole trajectory" << reset << std::endl;
        return;
      }
      whole_traj_gurobi.fillX();
      X_whole_out = whole_traj_gurobi.X_temp_;
      whole_traj_JPS = whole_jps_traj;
    }
    else
    {  
      state dummy;
      std::vector<state> dummy_vector;
      dummy_vector.push_back(dummy);
      whole_traj_gurobi.X_temp_ = dummy_vector;
    }

    vec_Vecf<3> JPSk_inside_sphere_tmp = pts_in_jps;
    bool thereIsIntersection2;
    motion_state.pos = getFirstCollisionJPS(JPSk_inside_sphere_tmp, &thereIsIntersection2, 2, 1);  

    bool needToComputeSafePath;
    int indexH = findIndexH(needToComputeSafePath);

    std::cout << "NeedToComputeSafePath=" << needToComputeSafePath << std::endl;

    if (bot_parameter_.use_faster == false)
    {
      needToComputeSafePath = true;
    }
    if (needToComputeSafePath == false)
    {
      k_safe = indexH;
      safe_traj_gurobi.X_temp_ = std::vector<state>();
    }
    else
    {
      mtx_X_U_temp.lock();
      k_safe = findIndexR(indexH);
      state R = whole_traj_gurobi.X_temp_[k_safe];
      mtx_X_U_temp.unlock();
      JPSk_inside_sphere_tmp[0] = R.pos;

      if (bot_parameter_.use_faster == false)
      {
        JPSk_inside_sphere_tmp[0] = A.pos;
      }

      vec_Vecf<3> JPS_safe = JPSk_inside_sphere_tmp;

      deleteVertexes(JPS_safe, bot_parameter_.max_poly_safe);
      motion_state.pos = JPS_safe[JPS_safe.size() - 1];

      jps_wrapper_obj.get_Safe_Zone(JPS_safe, UNKOWN_AND_OCCUPIED_SPACE, l_constraints_safe_, safe_op_polyhedron);

      Safe_traj_JPS = JPS_safe;

      bool isGinside = l_constraints_safe_[l_constraints_safe_.size() - 1].inside(GOAL_pos.pos);
      motion_state.pos = (isGinside == true) ? GOAL_pos.pos : motion_state.pos;

      state x0_safe;
      x0_safe = R;

      if (bot_parameter_.use_faster == false)
      {
        x0_safe = stateA_;
      }

      bool shouldForceFinalConstraint_for_Safe = (bot_parameter_.use_faster == false) ? true : false;

      if (l_constraints_safe_[0].inside(x0_safe.pos) == false)
      {
        std::cout << red << "First point of safe traj is outside" << reset << std::endl;
      }

      safe_traj_gurobi.setX0(x0_safe);
      safe_traj_gurobi.setXf(motion_state);  // only used to compute dt
      safe_traj_gurobi.setPolytopes(l_constraints_safe_);
      safe_traj_gurobi.setForceFinalConstraint(shouldForceFinalConstraint_for_Safe);
      MyTimer safe_gurobi_t(true);
      std::cout << "Calling Gurobi" << std::endl;
      bool solved_safe = safe_traj_gurobi.genNewTraj();

      if (solved_safe == false)
      {
        std::cout << red << "No solution found for the safe path" << reset << std::endl;
        return;
      }
      safe_traj_gurobi.fillX();
      X_safe_out = safe_traj_gurobi.X_temp_;
    }

    if (appendToPlan(k_end_whole, whole_traj_gurobi.X_temp_, k_safe, safe_traj_gurobi.X_temp_) != true)
    {
      return;
    }
  
    state F = plan_.back(); 
    double dist = (Final_goal_.pos - F.pos).norm();

    if (dist < bot_parameter_.goal_radius)
    {
      changeBotStatus(TurtlebotStatus::goal_in_map);
    }

    mtx_offsets.lock();

    int states_last_replan = ceil(replanCB_t.ElapsedMs() / (bot_parameter_.dc * 1000));                                                                          
    mtx_offsets.unlock();
    double new_init_whole = std::max(whole_traj_gurobi.factor_that_worked_ - bot_parameter_.gamma_whole, 1.0);
    double new_final_whole = whole_traj_gurobi.factor_that_worked_ + bot_parameter_.gammap_whole;
    whole_traj_gurobi.setFactorInitialAndFinalAndIncrement(new_init_whole, new_final_whole, bot_parameter_.increment_whole);
    double new_init_safe = std::max(safe_traj_gurobi.factor_that_worked_ - bot_parameter_.gamma_safe, 1.0);
    double new_final_safe = safe_traj_gurobi.factor_that_worked_ + bot_parameter_.gammap_safe;
    safe_traj_gurobi.setFactorInitialAndFinalAndIncrement(new_init_safe, new_final_safe, bot_parameter_.increment_safe);
    planner_initialized_ = true;

    std::cout<< "Replanning took " << replanCB_t.ElapsedMs() << " ms" << reset << std::endl;
    return;
  }

  bool getNextGoal(state& next_goal) 
  {
    if (initializedAllExceptPlanner() == false)
    {
      std::cout << "Not publishing new goal!!" << std::endl;
      return false;
    }

    mtx_goals.lock();
    mtx_plan_.lock();

    next_goal.setZero();
    next_goal = plan_.front();
    if (plan_.size() > 1)
    {
      plan_.pop_front();
    }
    getDesiredTheta(next_goal);
    previous_theta = next_goal.theta;
    mtx_goals.unlock();
    mtx_plan_.unlock();
    return true;
  }

  void State_getter(state& data)
  {
    mtx_state.lock();
    data = state_;
    mtx_state.unlock();
  }

  void getG(state& GOAL_pos) {
    GOAL_pos = temp_goal_inside_map;
  }

  int getTurtlebotStatus() {
    switch (turtlebot_state_)
      {
        case TurtlebotStatus::Searching:
          return 0;   
        case TurtlebotStatus::in_motion:
          return 1;
        case TurtlebotStatus::goal_in_map:
          return 2;
        case TurtlebotStatus::goal_reached:
          return 3;
      }
      return 3;
  }

private:
  state stateA_; 
  state state_;
  state temp_goal_inside_map; 
  state Final_goal_;
  state motion_state;
  std::mutex mtx_X_U_safe;
  std::mutex mtx_X_U;
  std::mutex mtx_planner_status_;
  parameters bot_parameter_;
  bool state_initialized_ = false;
  bool planner_initialized_ = false;
  vec_E<Polyhedron<3>> polyhedra_;
  std::vector<LinearConstraint3D> l_constraints_whole_;
  int deltaTime_ = 10;
  int indexR_ = 0; 
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_map_;       
  double dtheta_filtered = 0;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_frontier_;  
  bool kdtree_map_initialized_ = 0;
  bool kdtree_unk_initialized_ = 0;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_unk_;
  bool terminal_goal_initialized_ = false;
  int x_;  
  int y_;  
  int z_;  
  std::mutex mutex_obj_1;
  std::mutex mtx_state;
  std::mutex mtx_offsets;
  std::mutex mtx_plan_;
  int turtlebot_state_ = TurtlebotStatus::in_motion;
  int planner_status_ = CurrentStatus::intial_plan;
  std::mutex mtx_inst;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk_;
  std::mutex mtx_map; 
  std::vector<LinearConstraint3D> l_constraints_safe_;
  std::mutex mtx_unk; 
  std::mutex mtx_frontier;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map_;
  std::mutex mtx_goals;
  double previous_theta = 0.0;
  SolverGurobi whole_traj_gurobi;  
  SolverGurobi safe_traj_gurobi;   
  JPS_Wrapper jps_wrapper_obj; 
  std::mutex mtx_k;
  std::mutex mtx_X_U_temp;
  std::mutex mtx_G;
  std::mutex mtx_G_term;
  Eigen::Vector3d pos_old_;
  Eigen::Vector3d B_;
  bool to_land_ = false;
  bool JPSk_solved_ = false;
  std::deque<state> plan_;


  void theta(double diff, state& next_goal) 
  {
    saturate(diff, -bot_parameter_.dc * bot_parameter_.w_max, bot_parameter_.dc * bot_parameter_.w_max);
    double dtheta_not_filtered;
    dtheta_not_filtered = copysign(1, diff) * bot_parameter_.w_max;
    dtheta_filtered = (1 - bot_parameter_.alpha_filter_dyaw) * dtheta_not_filtered + bot_parameter_.alpha_filter_dyaw * dtheta_filtered;
    next_goal.dtheta = dtheta_filtered;
    next_goal.theta = previous_theta + dtheta_filtered * bot_parameter_.dc;
  }

  void getDesiredTheta(state& next_goal) {
    double diff = 0.0;
    double desired_yaw = 0.0;
    switch (turtlebot_state_)
    {
      case TurtlebotStatus::Searching:
        desired_yaw = atan2(Final_goal_.pos[1] - next_goal.pos[1], Final_goal_.pos[0] - next_goal.pos[0]);
        diff = desired_yaw - state_.theta;
        break;
      case TurtlebotStatus::in_motion:
      case TurtlebotStatus::goal_in_map:
        desired_yaw = atan2(motion_state.pos[1] - next_goal.pos[1], motion_state.pos[0] - next_goal.pos[0]);
        diff = desired_yaw - state_.theta;
        break;
      case TurtlebotStatus::goal_reached:
        next_goal.dtheta = 0.0;
        next_goal.theta = previous_theta;
        return;
    }

    angle_wrap(diff);
    if (fabs(diff) < 0.04 && turtlebot_state_ == TurtlebotStatus::Searching)
    {
      changeBotStatus(TurtlebotStatus::in_motion);
    }
    theta(diff, next_goal);
  }

  void createMoreVertexes(vec_Vecf<3>& path, double d) 
  {
    for (int j = 0; j < path.size() - 1; j++) {
      double dist = (path[j + 1] - path[j]).norm();
      int vertexes_to_add = floor(dist / d);
      Eigen::Vector3d v = (path[j + 1] - path[j]).normalized();
      // std::cout << "Vertexes to add=" << vertexes_to_add << std::endl;
      if (dist > d) {
        for (int i = 0; i < vertexes_to_add; i++) {
          path.insert(path.begin() + j + 1, path[j] + v * d);
          j = j + 1;
        }
      }
    }
  }

  int findIndexR(int indexH) 
  {
    Eigen::Vector2d posHk;
    posHk << whole_traj_gurobi.X_temp_[indexH].pos(0), whole_traj_gurobi.X_temp_[indexH].pos(1);
    int indexR = indexH;
    for (int i = 0; i <= indexH; i = i + 1) 
    { 
      Eigen::Vector2d vel;
      vel << whole_traj_gurobi.X_temp_[i].vel(0), whole_traj_gurobi.X_temp_[i].vel(1);  
      Eigen::Vector2d pos;
      pos << whole_traj_gurobi.X_temp_[i].pos(0), whole_traj_gurobi.X_temp_[i].pos(1);
      Eigen::Vector2d braking_distance = (vel.array() * (posHk - pos).array()).sign() * vel.array().square() / (2 * bot_parameter_.delta_a * bot_parameter_.a_max);

      bool thereWillBeCollision =(braking_distance.array() > (posHk - pos).cwiseAbs().array()).any();
      if (thereWillBeCollision) 
      {
        indexR = i;
        if (indexR == 0) {
          std::cout << "R was taken in A" << reset << std::endl;
        }
        break;
      }
    }
    return indexR;
  }

  int findIndexH(bool& needToComputeSafePath) 
  {
    int n = 1; 
    std::vector<int> pointIdxNKNSearch(n);
    std::vector<float> pointNKNSquaredDistance(n);
    needToComputeSafePath = false;
    mtx_unk.lock();
    mtx_X_U_temp.lock();
    int indexH = whole_traj_gurobi.X_temp_.size() - 1;
    for (int i = 0; i < whole_traj_gurobi.X_temp_.size(); i = i + 10) 
    { 
      Eigen::Vector3d temp = whole_traj_gurobi.X_temp_[i].pos;
      pcl::PointXYZ searchPoint(temp(0), temp(1), temp(2));
      if (kdtree_unk_.nearestKSearch(searchPoint, n, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) 
      {
        if (sqrt(pointNKNSquaredDistance[0]) < bot_parameter_.turtlebot_radius) 
        {
          needToComputeSafePath = true;
          indexH = (int)(bot_parameter_.delta_H * i);
          break;
        }
      }
    }
    mtx_unk.unlock();
    mtx_X_U_temp.unlock();
    return indexH;
  }

  bool ARisInFreeSpace(int index) {
    int n = 1;
    std::vector<int> pointIdxNKNSearch(n);
    std::vector<float> pointNKNSquaredDistance(n);
    bool isFree = true;
    mtx_unk.lock();
    mtx_X_U_temp.lock();
    for (int i = 0; i < index; i = i + 10)
    {
      Eigen::Vector3d temp = whole_traj_gurobi.X_temp_[i].pos;
      pcl::PointXYZ searchPoint(temp(0), temp(1), temp(2));
      if (kdtree_unk_.nearestKSearch(searchPoint, n, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        if (sqrt(pointNKNSquaredDistance[0]) < 0.2)
        {
          std::cout << "A->R collides, with d=" << sqrt(pointNKNSquaredDistance[0])<< ", radius_drone=" << bot_parameter_.turtlebot_radius << std::endl;
          isFree = false;
          break;
        }
      }
    }
    mtx_unk.unlock();
    mtx_X_U_temp.unlock();
    return isFree;
  }

  void changeBotStatus(int new_status) 
  {
    if (new_status == turtlebot_state_)
    {
      return;
    }
    // std::cout << "Changing TurtlebotStatus from ";
    // switch (turtlebot_state_)
    // {
    //   case TurtlebotStatus::Searching:
    //     std::cout << bold << "status_=Searching" << reset;
    //     break;
    //   case TurtlebotStatus::in_motion:
    //     std::cout << bold << "status_=in_motion" << reset;
    //     break;
    //   case TurtlebotStatus::goal_in_map:
    //     std::cout << bold << "status_=goal_in_map" << reset;
    //     break;
    //   case TurtlebotStatus::goal_reached:
    //     std::cout << bold << "status_=goal_reached" << reset;
    //     break;
    // }
    // std::cout << " to ";
    // switch (new_status)
    // {
    //   case TurtlebotStatus::Searching:
    //     std::cout << bold << "status_=Searching" << reset;
    //     break;
    //   case TurtlebotStatus::in_motion:
    //     std::cout << bold << "status_=in_motion" << reset;
    //     break;
    //   case TurtlebotStatus::goal_in_map:
    //     std::cout << bold << "status_=goal_in_map" << reset;
    //     break;
    //   case TurtlebotStatus::goal_reached:
    //     std::cout << bold << "status_=goal_reached" << reset;
    //     break;
    // }

    // std::cout << std::endl;

    turtlebot_state_ = new_status;
  }

  Eigen::Vector3d getFirstCollisionJPS(vec_Vecf<3>& path, bool* thereIsIntersection, int map, int type_return) 
  {
    vec_Vecf<3> original = path;
    Eigen::Vector3d first_element = path[0];
    Eigen::Vector3d last_search_point = path[0];
    Eigen::Vector3d inters = path[0];
    pcl::PointXYZ pcl_search_point = eigenPoint2pclPoint(path[0]);
    Eigen::Vector3d result;
    int n = 1;
    std::vector<int> id_map(n);
    std::vector<float> dist2_map(n);
    double r = 1000000;
    mtx_map.lock();
    mtx_unk.lock();
    int last_id = -1;
    int iteration = 0;
    while (path.size() > 0)
    {
      pcl_search_point = eigenPoint2pclPoint(path[0]);
      int number_of_neigh;
      if (map == 1)
      {
        number_of_neigh = kdtree_map_.nearestKSearch(pcl_search_point, n, id_map, dist2_map);
      }
      else
      {
        number_of_neigh = kdtree_unk_.nearestKSearch(pcl_search_point, n, id_map, dist2_map);
      }
      if (number_of_neigh > 0)
      {
        r = sqrt(dist2_map[0]);
        if (r < bot_parameter_.turtlebot_radius)
        {
          switch (type_return)
          {
            case 0:
              result = last_search_point;
              break;
            case 1:
              if (iteration == 0)
              { 
                Eigen::Vector3d temp;
                temp << original[0](0) + 0.01, original[0](1), original[0](2);
                path.clear();
                path.push_back(original[0]);
                path.push_back(temp);
                result = path[path.size() - 1];
              }
              else
              {
                int vertexes_eliminated_tmp = original.size() - path.size() + 1;
                original.erase(original.begin() + vertexes_eliminated_tmp,original.end()); 
                original.push_back(path[0]);
                reduceJPSbyDistance(original, bot_parameter_.turtlebot_radius);
                result = original[original.size() - 1];
                path = original;
              }
              break;
          }
          *thereIsIntersection = true;
          break;
        }
        bool no_points_outside_sphere = false;
        inters = getFirstIntersectionWithSphere(path, r, path[0], &last_id, &no_points_outside_sphere);
        if (no_points_outside_sphere == true)
        {
          *thereIsIntersection = false;
          result = first_element;
          if (type_return == 1)
          {
            result = original[original.size() - 1];
            path = original;
          }
          break;
        }
        last_search_point = path[0];
        path.erase(path.begin(), path.begin() + last_id + 1);
        path.insert(path.begin(), inters);
      }
      else
      {
        *thereIsIntersection = false;
        ROS_INFO("JPS provided doesn't intersect any obstacles, returning the first element of the path you gave me\n");
        result = first_element;
        if (type_return == 1)
        {
          result = original[original.size() - 1];
          path = original;
        }
        break;
      }
      iteration = iteration + 1;
    }
    mtx_map.unlock();
    mtx_unk.unlock();
    return result;
  }

  bool appendToPlan(int k_end_whole, const std::vector<state>& whole, int k_safe, const std::vector<state>& safe) {
    mtx_plan_.lock();
    bool output;
    int plan_size = plan_.size();
    if ((plan_size - 1 - k_end_whole) < 0)
    {
      std::cout << bold << red << "Already publised the point A" << reset << std::endl;
      output = false;
    }
    else
    {
      plan_.erase(plan_.end() - k_end_whole - 1, plan_.end());
      for (int i = 0; i <= k_safe; i++)
      {
        plan_.push_back(whole[i]);
      }
      for (int i = 0; i < safe.size(); i++)
      {
        plan_.push_back(safe[i]);
      }
      output = true;
    }

    mtx_plan_.unlock();
    return output;
  }

  bool initializedAllExceptPlanner() 
  {
    if (!state_initialized_ || !kdtree_map_initialized_ || !kdtree_unk_initialized_ || !terminal_goal_initialized_) {
      std::cout << "state_initialized_= " << state_initialized_ << std::endl;
      std::cout << "kdtree_map_initialized_= " << kdtree_map_initialized_ << std::endl;
      std::cout << "kdtree_unk_initialized_= " << kdtree_unk_initialized_ << std::endl;
      std::cout << "terminal_goal_initialized_= " << terminal_goal_initialized_ << std::endl;
      return false;
    }
    return true;
  }

  void print_status() 
  {
    switch (turtlebot_state_)
    {
      case TurtlebotStatus::Searching:
        std::cout << bold << "status_=Searching" << reset << std::endl;
        break;
      case TurtlebotStatus::in_motion:
        std::cout << bold << "status_=in_motion" << reset << std::endl;
        break;
      case TurtlebotStatus::goal_in_map:
        std::cout << bold << "status_=goal_in_map" << reset << std::endl;
        break;
      case TurtlebotStatus::goal_reached:
        std::cout << bold << "status_=goal_reached" << reset << std::endl;
        break;
    }
  }

};

class Local_Planner_Ros
{
public:
  Local_Planner_Ros(ros::NodeHandle nh) : nh_(nh) {
    bot_parameter_.use_ff = true;
    bot_parameter_.visual = true;
    bot_parameter_.dc = 0.02;
    bot_parameter_.goal_radius = 0.3;
    bot_parameter_.turtlebot_radius = 0.4;
    bot_parameter_.N_whole = 6;
    bot_parameter_.N_safe = 6;
    bot_parameter_.Ra = 4.0;
    bot_parameter_.w_max = 1.84;
    bot_parameter_.alpha_filter_dyaw = 0;
    bot_parameter_.z_ground = -0.2;
    bot_parameter_.z_max = 0.5;
    bot_parameter_.inflation_jps = 0.47;
    bot_parameter_.factor_jps = 1;
    bot_parameter_.v_max = 0.26;
    bot_parameter_.a_max = 1;
    bot_parameter_.j_max = 2;
    bot_parameter_.gamma_whole = 20;
    bot_parameter_.gammap_whole = 20;
    bot_parameter_.increment_whole = 1.0;
    bot_parameter_.gamma_safe = 20;
    bot_parameter_.gammap_safe = 20;
    bot_parameter_.increment_safe = 1.0;
    bot_parameter_.delta_a = 0.5;
    bot_parameter_.delta_H = 1.0;
    bot_parameter_.max_poly_whole = 3;
    bot_parameter_.max_poly_safe = 3;
    bot_parameter_.dist_max_vertexes = 1.5;
    bot_parameter_.gurobi_threads = 0;
    bot_parameter_.gurobi_verbose = 0;
    bot_parameter_.use_faster = true;
    bot_parameter_.wdx = 20.0;
    bot_parameter_.wdy = 20.0;
    bot_parameter_.wdz = 4.0;
    bot_parameter_.res = 0.15;
    bot_parameter_.is_ground_robot = true;
    bot_parameter_.goal_height = 1;
    bot_parameter_.force_goal_height = true;

    if ((bot_parameter_.N_safe <= bot_parameter_.max_poly_safe + 2) || (bot_parameter_.N_whole <= bot_parameter_.max_poly_whole + 2) || (bot_parameter_.factor_jps * bot_parameter_.res / 2.0 > bot_parameter_.inflation_jps)) {
      abort();
    }

    faster_ptr_ = std::unique_ptr<Local_Planner>(new Local_Planner(bot_parameter_));
    ROS_INFO("Planner initialized");

    pub_goal_ = nh_.advertise<snapstack_msgs::Goal>("goal", 1);
    pub_setpoint_ = nh_.advertise<visualization_msgs::Marker>("setpoint", 1);
    pub_intersectionI_ = nh_.advertise<visualization_msgs::Marker>("intersection_I", 1);
    pub_point_G_ = nh_.advertise<geometry_msgs::PointStamped>("point_G", 1);
    pub_point_G_term_ = nh_.advertise<geometry_msgs::PointStamped>("point_G_term", 1);
    pub_point_E_ = nh_.advertise<visualization_msgs::Marker>("point_E", 1);
    pub_point_R_ = nh_.advertise<visualization_msgs::Marker>("point_R", 1);
    pub_point_M_ = nh_.advertise<visualization_msgs::Marker>("point_M", 1);
    pub_point_H_ = nh_.advertise<visualization_msgs::Marker>("point_H", 1);
    pub_point_A_ = nh_.advertise<visualization_msgs::Marker>("point_A", 1);
    pub_actual_traj_ = nh_.advertise<visualization_msgs::Marker>("actual_traj", 1);
    pub_path_jps_whole_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps_whole", 1);
    pub_path_jps_safe_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps_safe", 1);
    poly_whole_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("poly_whole", 1, true);
    poly_safe_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("poly_safe", 1, true);
    pub_jps_inters_ = nh_.advertise<geometry_msgs::PointStamped>("jps_intersection", 1);
    pub_plan_state_ = nh_.advertise<std_msgs::Int8>("plan_state", 1);
    pub_traj_committed_colored_ = nh_.advertise<visualization_msgs::MarkerArray>("traj_committed_colored", 1);
    pub_traj_whole_colored_ = nh_.advertise<visualization_msgs::MarkerArray>("traj_whole_colored", 1);
    pub_traj_safe_colored_ = nh_.advertise<visualization_msgs::MarkerArray>("traj_safe_colored", 1);
    occup_grid_sub_.subscribe(nh_, "occup_grid", 1);
    unknown_grid_sub_.subscribe(nh_, "unknown_grid", 1);
    sync_.reset(new Sync(MySyncPolicy(1), occup_grid_sub_, unknown_grid_sub_));
    sync_->registerCallback(boost::bind(&Local_Planner_Ros::mapCB, this, _1, _2));
    sub_goal_ = nh_.subscribe("term_goal", 1, &Local_Planner_Ros::terminalGoalCB, this);
    sub_mode_ = nh_.subscribe("mode", 1, &Local_Planner_Ros::modeCB, this);
    sub_state_ = nh_.subscribe("state", 1, &Local_Planner_Ros::stateCB, this);
    pubCBTimer_ = nh_.createTimer(ros::Duration(bot_parameter_.dc), &Local_Planner_Ros::pubCB, this);
    replanCBTimer_ = nh_.createTimer(ros::Duration(bot_parameter_.dc), &Local_Planner_Ros::replanCB, this);
    occup_grid_sub_.unsubscribe();
    unknown_grid_sub_.unsubscribe();
    sub_state_.shutdown();
    pubCBTimer_.stop();
    replanCBTimer_.stop();
    setpoint_ = getMarkerSphere(0.35, ORANGE_TRANS);
    R_ = getMarkerSphere(0.35, ORANGE_TRANS);
    I_ = getMarkerSphere(0.35, YELLOW);
    E_ = getMarkerSphere(0.35, RED);
    motion_state = getMarkerSphere(0.35, BLUE);
    H_ = getMarkerSphere(0.35, GREEN);
    A_ = getMarkerSphere(0.35, RED);
    clearMarkerActualTraj();
  }

private:
  std::unique_ptr<Local_Planner> faster_ptr_;

  void pubTraj(const std::vector<state>& data, int type) 
  {
    nav_msgs::Path traj;
    traj.poses.clear();
    traj.header.stamp = ros::Time::now();
    traj.header.frame_id = world_name_;

    geometry_msgs::PoseStamped temp_path;

    for (int i = 0; i < data.size(); i = i + 8)
    {
      temp_path.pose.position.x = data[i].pos(0);
      temp_path.pose.position.y = data[i].pos(0);
      temp_path.pose.position.z = data[i].pos(0);
      temp_path.pose.orientation.w = 1;
      temp_path.pose.orientation.x = 0;
      temp_path.pose.orientation.y = 0;
      temp_path.pose.orientation.z = 0;
      traj.poses.push_back(temp_path);
    }
    clearMarkerColoredTraj();
    clearMarkerArray(&traj_committed_colored_, &pub_traj_committed_colored_);
    clearMarkerArray(&traj_whole_colored_, &pub_traj_whole_colored_);
    clearMarkerArray(&traj_safe_colored_, &pub_traj_safe_colored_);
    traj_committed_colored_ = stateVector2ColoredMarkerArray(data, type, bot_parameter_.v_max);
    pub_traj_committed_colored_.publish(traj_committed_colored_);
    traj_whole_colored_ = stateVector2ColoredMarkerArray(data, type, bot_parameter_.v_max);
    pub_traj_whole_colored_.publish(traj_whole_colored_);
    traj_safe_colored_ = stateVector2ColoredMarkerArray(data, type, bot_parameter_.v_max);
    pub_traj_safe_colored_.publish(traj_safe_colored_);
  }
  
  void terminalGoalCB(const geometry_msgs::PoseStamped& msg) 
  {
    state G_term;
    double height;
    // if (bot_parameter_.is_ground_robot)
    // {
    height = 0.2;
    // }
    // else if (bot_parameter_.force_goal_height)
    // {
    //   height = bot_parameter_.goal_height;
    // }
    // else
    // {
    //   height = msg.pose.position.z;
    // }
    G_term.setPos(msg.pose.position.x, msg.pose.position.y, height);
    faster_ptr_->setTerminalGoal(G_term);
    state GOAL_pos; 
    faster_ptr_->getG(GOAL_pos);
    pubState(G_term, pub_point_G_term_);
    pubState(GOAL_pos, pub_point_G_);
    clearMarkerActualTraj();
  }

  void pubState(const state& data, const ros::Publisher pub) 
  {
    geometry_msgs::PointStamped p;
    p.header.frame_id = world_name_;
    p.point = eigen2point(data.pos);
    pub.publish(p);
  }

  void stateCB(const snapstack_msgs::State& msg) 
  {
    state state_tmp;
    state_tmp.setPos(msg.pos.x, msg.pos.y, msg.pos.z);
    state_tmp.setVel(msg.vel.x, msg.vel.y, msg.vel.z);
    state_tmp.setAccel(0.0, 0.0, 0.0);
    double roll, pitch, theta;
    quaternion2Euler(msg.quat, roll, pitch, theta);
    state_tmp.setYaw(theta);
    faster_ptr_->updateState(state_tmp);
  }

  void modeCB(const custom_msgs::Mode& msg)
  {
    if (msg.mode != msg.GO)
    {  
      occup_grid_sub_.unsubscribe();
      unknown_grid_sub_.unsubscribe();
      sub_state_.shutdown();
      pubCBTimer_.stop();
      replanCBTimer_.stop();
      faster_ptr_->hard_reset();
    }
    else
    {  
      occup_grid_sub_.subscribe();
      unknown_grid_sub_.subscribe();
      sub_state_ = nh_.subscribe("state", 1, &Local_Planner_Ros::stateCB, this);  
      pubCBTimer_.start();
      replanCBTimer_.start();
    }
  }

  void pubCB(const ros::TimerEvent& e)
  {
    state next_goal;
    if (faster_ptr_->getNextGoal(next_goal))
    {
      snapstack_msgs::Goal quadGoal;
      quadGoal.p = eigen2rosvector(next_goal.pos);
      quadGoal.v = eigen2rosvector(next_goal.vel);
      quadGoal.a = eigen2rosvector(next_goal.accel);
      quadGoal.j = eigen2rosvector(next_goal.jerk);
      quadGoal.dyaw = next_goal.dtheta;
      quadGoal.yaw = next_goal.theta;
      quadGoal.header.stamp = ros::Time::now();
      quadGoal.header.frame_id = world_name_;
      pub_goal_.publish(quadGoal);
      setpoint_.header.stamp = ros::Time::now();
      setpoint_.pose.position.x = quadGoal.p.x;
      setpoint_.pose.position.y = quadGoal.p.y;
      setpoint_.pose.position.z = quadGoal.p.z;
      pub_setpoint_.publish(setpoint_);
      std_msgs::Int8 i;
      i.data = faster_ptr_->getTurtlebotStatus();
      pub_plan_state_.publish(i);
      
    }
  }

  void replanCB(const ros::TimerEvent& e)
  {
    if (ros::ok())
    {
      vec_Vecf<3> JPS_safe;
      vec_Vecf<3> whole_jps_traj;
      vec_E<Polyhedron<3>> poly_safe;
      vec_E<Polyhedron<3>> poly_whole;
      std::vector<state> X_safe;
      std::vector<state> X_whole;
      faster_ptr_->replan(JPS_safe, whole_jps_traj, poly_safe, poly_whole, X_safe, X_whole);
      clearJPSPathVisualization(2);
      publishJPSPath(JPS_safe, 4);
      publishJPSPath(whole_jps_traj, 3);
      publishPoly(poly_safe, 2);
      publishPoly(poly_whole, 1);
      pubTraj(X_safe, 5);
      pubTraj(X_whole, 4);
    }
  }

  void clearMarkerActualTraj() 
  {
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::DELETEALL;
    m.id = 0;
    m.scale.x = 0.02;
    m.scale.y = 0.04;
    m.scale.z = 1;
    pub_actual_traj_.publish(m);
    actual_trajID_ = 0;
  }

  void clearMarkerColoredTraj() 
  {
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::DELETEALL;
    m.id = 0;
    m.scale.x = 1;
    m.scale.y = 1;
    m.scale.z = 1;
    pub_actual_traj_.publish(m);
  }
  
  void mapCB(const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_map_ros,const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_unk_ros) 
  { 
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pcl2ptr_map_ros, *pclptr_map);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pcl2ptr_unk_ros, *pclptr_unk);
    faster_ptr_->updateMap(pclptr_map, pclptr_unk);
  }
  
  void pubActualTraj() 
  {
    static geometry_msgs::Point p_last = pointOrigin();
    state current_state;
    faster_ptr_->State_getter(current_state);
    Eigen::Vector3d act_pos = current_state.pos;
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.id = actual_trajID_ % 3000;
    actual_trajID_++;
    m.color = color(RED);
    m.scale.x = 0.15;
    m.scale.y = 0;
    m.scale.z = 0;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = world_name_;
    geometry_msgs::Point p;
    p = eigen2point(act_pos);
    m.points.push_back(p_last);
    m.points.push_back(p);
    pub_actual_traj_.publish(m);
    p_last = p;
  }
  
  void clearMarkerArray(visualization_msgs::MarkerArray* temp, ros::Publisher* publisher) 
  {
    if ((*temp).markers.size() == 0)
    {
      return;
    }
    int id_begin = (*temp).markers[0].id;
    for (int i = 0; i < (*temp).markers.size(); i++)
    {
      visualization_msgs::Marker m;
      m.type = visualization_msgs::Marker::ARROW;
      m.action = visualization_msgs::Marker::DELETE;
      m.id = i + id_begin;
      (*temp).markers[i] = m;
    }
    (*publisher).publish(*temp);
    (*temp).markers.clear();
  }

  void publishJPSPath(vec_Vecf<3>& path, int i) {
    clearJPSPathVisualization(i);
    switch (i)
    {
      case 3:
        vectorOfVectors2MarkerArray(path, &path_jps_whole_, color(GREEN));
        pub_path_jps_whole_.publish(path_jps_whole_);
        break;
      case 4:
        vectorOfVectors2MarkerArray(path, &path_jps_safe_, color(YELLOW));
        pub_path_jps_safe_.publish(path_jps_safe_);
        break;
    }
  }


  void clearJPSPathVisualization(int i) {
    switch (i)
  {
    case 3:
      clearMarkerArray(&path_jps_whole_, &pub_path_jps_whole_);
      break;
    case 4:
      clearMarkerArray(&path_jps_safe_, &pub_path_jps_safe_);
      break;
  }
  }

  void pubJPSIntersection(Eigen::Vector3d& inters) {
    geometry_msgs::PointStamped p;
    p.header.frame_id = world_name_;
    p.point = eigen2point(inters);
    pub_jps_inters_.publish(p);
  }

  void publishPoly(const vec_E<Polyhedron<3>>& poly, int type) {
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly);
    poly_msg.header.frame_id = world_name_;

    switch (type)
    {
      case 2:
        poly_safe_pub_.publish(poly_msg);
        break;
      case 1:
        poly_whole_pub_.publish(poly_msg);
        break;
    }
  }

  std::string world_name_ = "world";
  visualization_msgs::Marker R_;
  visualization_msgs::Marker I_;
  visualization_msgs::Marker E_;
  visualization_msgs::Marker motion_state;
  visualization_msgs::Marker H_;
  visualization_msgs::Marker A_;
  visualization_msgs::Marker setpoint_;
  ros::NodeHandle nh_;
  ros::Publisher pub_goal_jackal_;
  ros::Publisher pub_point_G_;
  ros::Publisher pub_point_G_term_;
  ros::Publisher pub_goal_;
  ros::Publisher pub_setpoint_;
  ros::Publisher pub_actual_traj_;
  ros::Publisher pub_path_jps_safe_;
  ros::Publisher pub_path_jps_whole_;
  ros::Publisher pub_intersectionI_;
  ros::Publisher pub_plan_state_;
  ros::Publisher pub_point_R_;
  ros::Publisher pub_point_M_;
  ros::Publisher pub_point_E_;
  ros::Publisher pub_point_H_;
  ros::Publisher pub_point_A_;
  ros::Publisher pub_traj_committed_colored_;
  ros::Publisher pub_traj_whole_colored_;
  ros::Publisher pub_traj_safe_colored_;
  ros::Publisher pub_planning_vis_;
  ros::Publisher pub_intersec_points_;
  ros::Publisher pub_jps_inters_;
  ros::Publisher pub_samples_safe_path_;
  ros::Publisher pub_log_;
  ros::Publisher poly_whole_pub_;
  ros::Publisher poly_safe_pub_;
  ros::Subscriber sub_goal_;
  ros::Subscriber sub_state_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_mode_;
  ros::Subscriber sub_vicon_;
  ros::Subscriber sub_frontier_;
  ros::Timer pubCBTimer_;
  ros::Timer replanCBTimer_;
  parameters bot_parameter_;  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener* tfListener;
  std::string name_drone_;
  visualization_msgs::MarkerArray trajs_sphere_;  
  visualization_msgs::MarkerArray path_jps1_;
  visualization_msgs::MarkerArray path_jps2_;
  visualization_msgs::MarkerArray path_jps2_fix_;
  visualization_msgs::MarkerArray path_jps_safe_;
  visualization_msgs::MarkerArray path_jps_whole_;
  visualization_msgs::MarkerArray traj_committed_colored_;
  visualization_msgs::MarkerArray traj_whole_colored_;
  visualization_msgs::MarkerArray traj_safe_colored_;
  visualization_msgs::MarkerArray intersec_points_;
  visualization_msgs::MarkerArray samples_safe_path_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> occup_grid_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> unknown_grid_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  int actual_trajID_ = 0;
};


int main(int argc, char **argv)
{
  // Initializes ROS, and sets up a node
  ros::init(argc, argv, "faster");
  ros::NodeHandle nh("~");
  ros::CallbackQueue custom_queue1;
  ros::CallbackQueue custom_queue2;
  nh.setCallbackQueue(&custom_queue1);
  nh.setCallbackQueue(&custom_queue2);

  Local_Planner_Ros Local_Planner_Ros(nh);

  ros::AsyncSpinner spinner1(0, &custom_queue1);
  ros::AsyncSpinner spinner2(0, &custom_queue2);
  spinner1.start();  
  spinner2.start();  
  ros::spin(); 
  ros::waitForShutdown();
  return 0;
}