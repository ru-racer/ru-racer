#pragma once

#include "ru_racer/bike_model.hpp"
#include "ru_racer/planner_types.hpp"

#include <random>

namespace ru_racer {

// C++ port of `matlab/RRTBike.m` + loop logic in `matlab/RRTSpars_1.m`
// focusing on *exact* Spars-RRT/Spars-RRT* behavior (Drain pruning + randomized neighbor selection).
struct SparseRrtBikeParams {
  int max_iterations = 50000;
  int max_nodes = 50000;

  // Matches RRTBike constants
  double delta_goal = 0.4;
  double delta_drain = 0.3;
  double max_step = 0.2;

  // Input bounds from MATLAB
  double delta_max = M_PI / 9.0;
  double sfx_max = 0.1;

  // XY_BOUNDARY from MATLAB (default matches `RRTBike.m`)
  Vec<BikeModel::NX> x_min{ -2.6, -0.8, -2.0 * M_PI, 0.5, -0.5, -5.0 };
  Vec<BikeModel::NX> x_max{  0.3,  3.5,  2.5 * M_PI, 3.5,  0.5,  5.0 };

  // MATLAB varies this each iteration: delta_near = 1 + (.5 - rand)
  // We'll keep the same formula when iterating.
  double delta_near_base = 1.0;

  // "type" in MATLAB affects steering + extra pruning when goal reached:
  // - 'o' : open-loop biased random steering
  // - 'p' : enable pruning of high-cost leaf nodes after reaching goal
  char type = 'o';

  // Comparison variants:
  bool use_best_parent = true; // RRT* parent selection
  bool use_rewire = false;     // RRT* rewiring
  bool use_drain = true;       // Spars pruning
};

class SparseRrtBikePlanner {
 public:
  using NodeT = PlannerNode<BikeModel::NX, BikeModel::NU>;
  using ResultT = PlannerResult<BikeModel::NX, BikeModel::NU>;

  SparseRrtBikePlanner(BikeModel model, Vec<BikeModel::NX> start, Vec<BikeModel::NX> goal, SparseRrtBikeParams p);

  void seed(std::uint64_t s) { rng_.seed(s); }

  ResultT plan();

 private:
  // MATLAB helpers
  static double solvePI(double x);

  Vec<BikeModel::NX> sample();
  int nearest(const Vec<BikeModel::NX>& node) const;
  std::vector<int> neighbors(const Vec<BikeModel::NX>& new_node, int nearest_ind, double delta_near);
  int findNewParent(const std::vector<int>& neigh, const Vec<BikeModel::NX>& new_node, int nearest_ind) const;

  bool steer(int from_ind, const Vec<BikeModel::NX>& random_node, Vec<BikeModel::NX>& out_new_state, Vec<BikeModel::NU>& out_u);

  double evalCost(int from_ind, const Vec<BikeModel::NX>& new_node) const;
  double evalCostEst(int from_ind, const Vec<BikeModel::NX>& new_node) const;
  double distToGoal(const Vec<BikeModel::NX>& x) const;

  int addNode(int parent_ind, const Vec<BikeModel::NX>& new_node, const Vec<BikeModel::NU>& u_from_parent);

  void drain(int new_ind, int nearest_ind);
  void removeNode(int remove_ind);

  void rewire(int new_ind, const std::vector<int>& neigh, int min_ind);

  BikeModel model_;
  Vec<BikeModel::NX> start_{};
  Vec<BikeModel::NX> goal_{};
  SparseRrtBikeParams p_{};

  std::mt19937_64 rng_{1234};
  std::uniform_real_distribution<double> uni_{0.0, 1.0};

  ResultT res_{};
};

} // namespace ru_racer

