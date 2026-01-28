#pragma once

#include "ru_racer/bike_model.hpp"
#include "ru_racer/math.hpp"

#include <cstddef>
#include <random>
#include <vector>

namespace ru_racer {

struct RrtStarParams {
  int max_nodes = 50000;
  int max_iterations = 50000;

  double delta_goal = 0.4;
  double max_step = 0.2;      // seconds of forward sim for each edge
  double goal_sample_rate = 0.0; // probability of sampling the goal directly

  // Neighbor radius: r(n) = min(r_max, gamma * sqrt(log(n)/n)).
  // This is a pragmatic choice (your MATLAB uses a randomized threshold).
  double neighbor_gamma = 1.0;
  double neighbor_r_max = 1.0;

  // Control bounds (delta, sfx)
  double delta_max = M_PI / 9.0;
  double sfx_max = 0.1;

  // State sampling bounds (min/max for 6-state)
  Vec<BikeModel::NX> x_min{ -2.6, -0.8, -2.0 * M_PI, 0.5, -0.5, -5.0 };
  Vec<BikeModel::NX> x_max{  0.3,  3.5,  2.5 * M_PI, 3.5,  0.5,  5.0 };
};

struct RrtNode {
  Vec<BikeModel::NX> x{};
  Vec<BikeModel::NU> u_from_parent{}; // input applied from parent to reach x
  int parent = -1;
  double cost_from_parent = 0.0;
  double cum_cost = 0.0;
  double time_from_parent = 0.0;
  double cum_time = 0.0;
  double dist_to_goal = 0.0;
};

class RrtStarPlanner {
 public:
  RrtStarPlanner(BikeModel model, Vec<BikeModel::NX> start, Vec<BikeModel::NX> goal, RrtStarParams p);

  void seed(std::uint64_t s) { rng_.seed(s); }

  // Runs up to max_iterations (or until a goal node is found and "improved" some).
  void plan();

  const std::vector<RrtNode>& nodes() const { return nodes_; }
  const std::vector<int>& goalNodes() const { return goal_nodes_; }

  // Returns best goal node index, or -1 if none.
  int bestGoalNode() const;

  // Returns path as node indices from start (root=0) to target node (inclusive).
  std::vector<int> extractPath(int target_node) const;

 private:
  Vec<BikeModel::NX> sample();
  int nearest(const Vec<BikeModel::NX>& x) const;
  std::vector<int> near(const Vec<BikeModel::NX>& x) const;
  double metric(const Vec<BikeModel::NX>& a, const Vec<BikeModel::NX>& b) const;
  double edgeCostTimeApprox(const Vec<BikeModel::NX>& a, const Vec<BikeModel::NX>& b) const;
  double distToGoalScore(const Vec<BikeModel::NX>& x) const;

  bool steer(int parent_idx, const Vec<BikeModel::NX>& target, RrtNode& out_new);
  bool steerClosedLoop(int parent_idx, const Vec<BikeModel::NX>& target, RrtNode& out_new);

  void rewire(int new_idx, const std::vector<int>& near_ids);

  BikeModel model_;
  Vec<BikeModel::NX> start_{};
  Vec<BikeModel::NX> goal_{};
  RrtStarParams p_{};

  std::mt19937_64 rng_{1234};
  std::uniform_real_distribution<double> uni_{0.0, 1.0};

  std::vector<RrtNode> nodes_;
  std::vector<int> goal_nodes_;
};

} // namespace ru_racer

