#include "ru_racer/rrt_star.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ru_racer {

RrtStarPlanner::RrtStarPlanner(BikeModel model, Vec<BikeModel::NX> start, Vec<BikeModel::NX> goal, RrtStarParams p)
    : model_(std::move(model)), start_(start), goal_(goal), p_(p) {
  model_.setGoal(goal_);
  model_.setBounds(p_.x_min, p_.x_max);

  nodes_.reserve(static_cast<std::size_t>(p_.max_nodes));
  RrtNode root{};
  root.x = start_;
  root.parent = -1;
  root.cum_cost = 0.0;
  root.cum_time = 0.0;
  root.dist_to_goal = distToGoalScore(start_);
  nodes_.push_back(root);
}

double RrtStarPlanner::metric(const Vec<BikeModel::NX>& a, const Vec<BikeModel::NX>& b) const {
  // Roughly mimic MATLAB: sum over i=1..4 with special handling for i==3.
  // We'll use: (x,y,vx) Euclidean + heading proxy using angle to target.
  const double dx = a[0] - b[0];
  const double dy = a[1] - b[1];
  const double dvx = a[3] - b[3];
  const double heading_to = std::atan2(b[1] - a[1], b[0] - a[0]);
  const double dpsi = std::abs(heading_to);
  return std::sqrt(dx * dx + dy * dy + dvx * dvx + dpsi * dpsi);
}

double RrtStarPlanner::edgeCostTimeApprox(const Vec<BikeModel::NX>& a, const Vec<BikeModel::NX>& b) const {
  const double dist = std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]));
  const double vmean = 0.5 * (std::abs(a[3]) + std::abs(b[3]));
  return dist / std::max(0.1, vmean);
}

double RrtStarPlanner::distToGoalScore(const Vec<BikeModel::NX>& x) const {
  // Similar to RRTBike: norm([goal(1:2)-x(1:2); w_v*(goal(4)-x(4)); w_psi*(wrap(goal(3)-x(3))); w_heading*atan2(...) ])
  const double dx = goal_[0] - x[0];
  const double dy = goal_[1] - x[1];
  const double dv = 0.2 * (goal_[3] - x[3]);
  const double dpsi = 0.2 * wrapAngle(goal_[2] - x[2]);
  const double dheading = 0.0 * std::abs(std::atan2(dy, dx)); // keep 0 by default
  return std::sqrt(dx * dx + dy * dy + dv * dv + dpsi * dpsi + dheading * dheading);
}

Vec<BikeModel::NX> RrtStarPlanner::sample() {
  if (uni_(rng_) < p_.goal_sample_rate) return goal_;
  Vec<BikeModel::NX> x{};
  for (std::size_t i = 0; i < BikeModel::NX; ++i) {
    const double r = uni_(rng_);
    x[i] = p_.x_min[i] + r * (p_.x_max[i] - p_.x_min[i]);
  }
  return x;
}

int RrtStarPlanner::nearest(const Vec<BikeModel::NX>& x) const {
  double best = std::numeric_limits<double>::infinity();
  int best_i = 0;
  for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
    const double d = metric(nodes_[static_cast<std::size_t>(i)].x, x);
    if (d < best) {
      best = d;
      best_i = i;
    }
  }
  return best_i;
}

std::vector<int> RrtStarPlanner::near(const Vec<BikeModel::NX>& x) const {
  const int n = static_cast<int>(nodes_.size());
  const double n_d = std::max(2.0, static_cast<double>(n));
  const double r = std::min(p_.neighbor_r_max, p_.neighbor_gamma * std::sqrt(std::log(n_d) / n_d));

  std::vector<int> out;
  out.reserve(64);
  for (int i = 0; i < n; ++i) {
    if (metric(nodes_[static_cast<std::size_t>(i)].x, x) <= r) out.push_back(i);
  }
  return out;
}

bool RrtStarPlanner::steer(int parent_idx, const Vec<BikeModel::NX>& /*target*/, RrtNode& out_new) {
  // Random constant input for a short horizon (like MATLAB Steer for type 'o').
  std::uniform_real_distribution<double> du(-p_.delta_max, p_.delta_max);
  std::uniform_real_distribution<double> su(-p_.sfx_max, p_.sfx_max);
  Vec<BikeModel::NU> u{du(rng_), su(rng_)};

  model_.reset(nodes_[static_cast<std::size_t>(parent_idx)].x, Vec<BikeModel::NU>{0.0, 0.0});
  const auto x_end = model_.simulateConstantInput(u, p_.max_step);
  if (!model_.isFeasible()) return false;

  out_new.x = x_end;
  out_new.u_from_parent = u;
  out_new.parent = parent_idx;
  out_new.time_from_parent = model_.lastCost();
  out_new.cost_from_parent = edgeCostTimeApprox(nodes_[static_cast<std::size_t>(parent_idx)].x, x_end);
  out_new.cum_cost = nodes_[static_cast<std::size_t>(parent_idx)].cum_cost + out_new.cost_from_parent;
  out_new.cum_time = nodes_[static_cast<std::size_t>(parent_idx)].cum_time + out_new.time_from_parent;
  out_new.dist_to_goal = distToGoalScore(x_end);
  return true;
}

bool RrtStarPlanner::steerClosedLoop(int parent_idx, const Vec<BikeModel::NX>& target, RrtNode& out_new) {
  model_.reset(nodes_[static_cast<std::size_t>(parent_idx)].x, Vec<BikeModel::NU>{0.0, 0.0});
  const auto x_end = model_.simulateClosedLoopTo(target, p_.max_step);
  if (!model_.isFeasible()) return false;

  out_new.x = x_end;
  out_new.u_from_parent = model_.input();
  out_new.parent = parent_idx;
  out_new.time_from_parent = model_.lastCost();
  out_new.cost_from_parent = edgeCostTimeApprox(nodes_[static_cast<std::size_t>(parent_idx)].x, x_end);
  out_new.cum_cost = nodes_[static_cast<std::size_t>(parent_idx)].cum_cost + out_new.cost_from_parent;
  out_new.cum_time = nodes_[static_cast<std::size_t>(parent_idx)].cum_time + out_new.time_from_parent;
  out_new.dist_to_goal = distToGoalScore(x_end);
  return true;
}

void RrtStarPlanner::rewire(int new_idx, const std::vector<int>& near_ids) {
  // Try to improve each neighbor by routing through new_idx using a CL steer check.
  for (int nid : near_ids) {
    if (nid == 0 || nid == new_idx) continue;
    const double tentative = nodes_[static_cast<std::size_t>(new_idx)].cum_cost +
                             edgeCostTimeApprox(nodes_[static_cast<std::size_t>(new_idx)].x, nodes_[static_cast<std::size_t>(nid)].x);
    if (tentative + 1e-9 >= nodes_[static_cast<std::size_t>(nid)].cum_cost) continue;

    RrtNode test{};
    if (!steerClosedLoop(new_idx, nodes_[static_cast<std::size_t>(nid)].x, test)) continue;
    const double improved = nodes_[static_cast<std::size_t>(new_idx)].cum_cost +
                            edgeCostTimeApprox(nodes_[static_cast<std::size_t>(new_idx)].x, test.x);
    if (improved + 1e-9 >= nodes_[static_cast<std::size_t>(nid)].cum_cost) continue;

    nodes_[static_cast<std::size_t>(nid)].parent = new_idx;
    nodes_[static_cast<std::size_t>(nid)].u_from_parent = test.u_from_parent;
    nodes_[static_cast<std::size_t>(nid)].cost_from_parent =
        edgeCostTimeApprox(nodes_[static_cast<std::size_t>(new_idx)].x, nodes_[static_cast<std::size_t>(nid)].x);
    nodes_[static_cast<std::size_t>(nid)].cum_cost = improved;
    nodes_[static_cast<std::size_t>(nid)].time_from_parent = test.time_from_parent;
    nodes_[static_cast<std::size_t>(nid)].cum_time = nodes_[static_cast<std::size_t>(new_idx)].cum_time + test.time_from_parent;
    // Note: We do NOT propagate cost changes to descendants (full RRT* would). This keeps code simple and stable.
  }
}

void RrtStarPlanner::plan() {
  for (int it = 0; it < p_.max_iterations; ++it) {
    if (static_cast<int>(nodes_.size()) >= p_.max_nodes) break;

    const auto x_rand = sample();
    const int near_idx = nearest(x_rand);
    auto near_ids = near(x_rand);

    // Pick best parent among near nodes by estimated edge cost.
    int best_parent = near_idx;
    double best_cost = nodes_[static_cast<std::size_t>(near_idx)].cum_cost +
                       edgeCostTimeApprox(nodes_[static_cast<std::size_t>(near_idx)].x, x_rand);
    for (int cand : near_ids) {
      const double c = nodes_[static_cast<std::size_t>(cand)].cum_cost +
                       edgeCostTimeApprox(nodes_[static_cast<std::size_t>(cand)].x, x_rand);
      if (c < best_cost) {
        best_cost = c;
        best_parent = cand;
      }
    }

    RrtNode new_node{};
    // Mix of random-input and closed-loop steering helps exploration + connection.
    const bool ok = (uni_(rng_) < 0.7) ? steer(best_parent, x_rand, new_node) : steerClosedLoop(best_parent, x_rand, new_node);
    if (!ok) continue;

    const int new_idx = static_cast<int>(nodes_.size());
    nodes_.push_back(new_node);

    if (new_node.dist_to_goal < p_.delta_goal) goal_nodes_.push_back(new_idx);

    rewire(new_idx, near_ids);
  }
}

int RrtStarPlanner::bestGoalNode() const {
  if (goal_nodes_.empty()) return -1;
  int best = goal_nodes_.front();
  double best_cost = nodes_[static_cast<std::size_t>(best)].cum_cost;
  for (int i : goal_nodes_) {
    const double c = nodes_[static_cast<std::size_t>(i)].cum_cost;
    if (c < best_cost) {
      best_cost = c;
      best = i;
    }
  }
  return best;
}

std::vector<int> RrtStarPlanner::extractPath(int target_node) const {
  if (target_node < 0 || target_node >= static_cast<int>(nodes_.size())) return {};
  std::vector<int> path;
  int cur = target_node;
  while (cur >= 0) {
    path.push_back(cur);
    cur = nodes_[static_cast<std::size_t>(cur)].parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

} // namespace ru_racer

