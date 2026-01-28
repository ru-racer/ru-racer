#include "ru_racer/sparse_rrt_bike.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ru_racer {

SparseRrtBikePlanner::SparseRrtBikePlanner(BikeModel model,
                                           Vec<BikeModel::NX> start,
                                           Vec<BikeModel::NX> goal,
                                           SparseRrtBikeParams p)
    : model_(std::move(model)), start_(start), goal_(goal), p_(p) {
  model_.setGoal(goal_);
  model_.setBounds(p_.x_min, p_.x_max);
}

double SparseRrtBikePlanner::solvePI(double x) {
  // MATLAB:
  // if x>2*pi, x-2*pi; elseif x<-2*pi, x+2*pi; else x;
  const double twoPi = 2.0 * M_PI;
  if (x > twoPi) return x - twoPi;
  if (x < -twoPi) return x + twoPi;
  return x;
}

Vec<BikeModel::NX> SparseRrtBikePlanner::sample() {
  Vec<BikeModel::NX> node{};
  for (std::size_t i = 0; i < BikeModel::NX; ++i) {
    const double r = uni_(rng_);
    node[i] = p_.x_min[i] + r * (p_.x_max[i] - p_.x_min[i]);
  }
  return node;
}

int SparseRrtBikePlanner::nearest(const Vec<BikeModel::NX>& node) const {
  // MATLAB distance uses first 4 components, special case for i==3.
  double best = std::numeric_limits<double>::infinity();
  int best_i = 0;
  for (int i = 0; i < static_cast<int>(res_.nodes.size()); ++i) {
    double d2 = 0.0;
    const auto& xi = res_.nodes[static_cast<std::size_t>(i)].x;
    for (int k = 0; k < 4; ++k) {
      if (k == 2) {
        const double ang = std::atan2(node[1] - xi[1], node[0] - xi[0]);
        d2 += 1.0 * std::abs(ang);
      } else {
        const double dd = xi[static_cast<std::size_t>(k)] - node[static_cast<std::size_t>(k)];
        d2 += dd * dd;
      }
    }
    const double d = std::sqrt(d2);

    // MATLAB: penalize goal_reached nodes if >1
    bool is_goal = false;
    if (res_.goal_nodes.size() > 1) {
      for (int gid : res_.goal_nodes) {
        if (gid == i) {
          is_goal = true;
          break;
        }
      }
    }
    const double dp = is_goal ? (d + 100.0) : d;
    if (dp < best) {
      best = dp;
      best_i = i;
    }
  }
  return best_i;
}

std::vector<int> SparseRrtBikePlanner::neighbors(const Vec<BikeModel::NX>& new_node, int nearest_ind, double delta_near) {
  // MATLAB:
  // dist_vec computed like Nearest, then Neighbors = find(dist_vec < delta_near*rand)
  std::vector<int> neigh;
  neigh.reserve(128);

  const double thr = delta_near * uni_(rng_);
  for (int i = 0; i < static_cast<int>(res_.nodes.size()); ++i) {
    double d2 = 0.0;
    const auto& xi = res_.nodes[static_cast<std::size_t>(i)].x;
    for (int k = 0; k < 4; ++k) {
      if (k == 2) {
        const double ang = std::atan2(new_node[1] - xi[1], new_node[0] - xi[0]);
        d2 += 1.0 * std::abs(ang);
      } else {
        const double dd = xi[static_cast<std::size_t>(k)] - new_node[static_cast<std::size_t>(k)];
        d2 += dd * dd;
      }
    }
    if (d2 < thr * thr && i != nearest_ind) neigh.push_back(i);
  }
  return neigh;
}

double SparseRrtBikePlanner::evalCost(int from_ind, const Vec<BikeModel::NX>& new_node) const {
  // MATLAB: norm(xy)/mean([vx_from, vx_new])
  const auto& x = res_.nodes[static_cast<std::size_t>(from_ind)].x;
  const double dist = std::sqrt((x[0] - new_node[0]) * (x[0] - new_node[0]) + (x[1] - new_node[1]) * (x[1] - new_node[1]));
  const double vmean = 0.5 * (x[3] + new_node[3]);
  return dist / std::max(1e-6, vmean);
}

double SparseRrtBikePlanner::evalCostEst(int from_ind, const Vec<BikeModel::NX>& new_node) const {
  const auto& x = res_.nodes[static_cast<std::size_t>(from_ind)].x;
  const double ang = std::abs(std::atan2(new_node[1] - x[1], new_node[0] - x[0]));
  if (ang < 0.6) {
    return evalCost(from_ind, new_node);
  }
  return 5.0 * std::abs(ang) / std::max(1e-6, x[3]);
}

double SparseRrtBikePlanner::distToGoal(const Vec<BikeModel::NX>& x) const {
  // MATLAB:
  // norm([goal(1:2)-x(1:2); .2*(goal(4)-x(4)); .2*solvePI(goal(3)-x(3)); 0*atan2(...)])
  const double dx = goal_[0] - x[0];
  const double dy = goal_[1] - x[1];
  const double dv = 0.2 * (goal_[3] - x[3]);
  const double dpsi = 0.2 * solvePI(goal_[2] - x[2]);
  const double dheading = 0.0 * std::abs(std::atan2(goal_[1] - x[1], goal_[0] - x[0]));
  return std::sqrt(dx * dx + dy * dy + dv * dv + dpsi * dpsi + dheading * dheading);
}

int SparseRrtBikePlanner::findNewParent(const std::vector<int>& neigh, const Vec<BikeModel::NX>& new_node, int nearest_ind) const {
  int parent_ind = nearest_ind;
  double min_cost = res_.nodes[static_cast<std::size_t>(nearest_ind)].cum_cost + evalCostEst(nearest_ind, new_node);
  for (int ni : neigh) {
    const double tc = res_.nodes[static_cast<std::size_t>(ni)].cum_cost + evalCostEst(ni, new_node);
    const int succ = res_.nodes[static_cast<std::size_t>(ni)].success;
    if (tc < min_cost && succ > -8 && succ < 20) {
      min_cost = tc;
      parent_ind = ni;
    }
  }
  return parent_ind;
}

bool SparseRrtBikePlanner::steer(int from_ind,
                                 const Vec<BikeModel::NX>& random_node,
                                 Vec<BikeModel::NX>& out_new_state,
                                 Vec<BikeModel::NU>& out_u) {
  model_.reset(res_.nodes[static_cast<std::size_t>(from_ind)].x, Vec<BikeModel::NU>{0.0, 0.0});

  const double a = uni_(rng_);
  if (a < 0.5 || p_.type == 'o') {
    // random input in bounds
    std::uniform_real_distribution<double> du(-p_.delta_max, p_.delta_max);
    std::uniform_real_distribution<double> su(-p_.sfx_max, p_.sfx_max);
    out_u = Vec<BikeModel::NU>{du(rng_), su(rng_)};
    model_.simulateConstantInput(out_u, p_.max_step);
  } else if (a > 0.7) {
    out_u = model_.controllerTo(random_node);
    model_.simulateConstantInput(out_u, p_.max_step);
  } else if (res_.nodes[static_cast<std::size_t>(res_.best_node)].dist_to_goal > 2.4) {
    Vec<BikeModel::NX> semi{-0.1, 2.7, M_PI * 0.1, 5.0, 0.0, 0.0};
    const auto u0 = model_.controllerTo(semi);
    const double s = 0.5 + 0.5 * uni_(rng_);
    out_u = Vec<BikeModel::NU>{s * u0[0], s * u0[1]};
    model_.simulateConstantInput(out_u, p_.max_step);
  } else {
    const auto u0 = model_.controllerTo(goal_);
    const double s = 0.5 + 0.5 * uni_(rng_);
    out_u = Vec<BikeModel::NU>{s * u0[0], s * u0[1]};
    model_.simulateConstantInput(out_u, p_.max_step);
  }

  const bool feasible = model_.isFeasible();
  // MATLAB updates success(from_ind)
  if (feasible) {
    res_.nodes[static_cast<std::size_t>(from_ind)].success += 1;
  } else {
    res_.nodes[static_cast<std::size_t>(from_ind)].success -= 1;
  }

  out_new_state = model_.state();
  return feasible;
}

int SparseRrtBikePlanner::addNode(int parent_ind, const Vec<BikeModel::NX>& new_node, const Vec<BikeModel::NU>& u_from_parent) {
  // MATLAB: reject if newcost > min(cumcost(goal_reached))
  if (!res_.goal_nodes.empty()) {
    double best_goal_cost = std::numeric_limits<double>::infinity();
    for (int gid : res_.goal_nodes) {
      best_goal_cost = std::min(best_goal_cost, res_.nodes[static_cast<std::size_t>(gid)].cum_cost);
    }
    const double newcost = res_.nodes[static_cast<std::size_t>(parent_ind)].cum_cost + evalCost(parent_ind, new_node);
    if (newcost > best_goal_cost) return 0;
  }

  if (static_cast<int>(res_.nodes.size()) >= p_.max_nodes) return 0;

  NodeT n{};
  n.x = new_node;
  n.u_from_parent = u_from_parent;
  n.parent = parent_ind;
  n.children = 0;
  n.cost_from_parent = evalCost(parent_ind, new_node);
  n.cum_cost = res_.nodes[static_cast<std::size_t>(parent_ind)].cum_cost + n.cost_from_parent;
  n.time_from_parent = model_.lastCost();
  n.cum_time = res_.nodes[static_cast<std::size_t>(parent_ind)].cum_time + n.time_from_parent;
  n.dist_to_goal = distToGoal(new_node);

  const int new_ind = static_cast<int>(res_.nodes.size());
  res_.nodes.push_back(n);
  res_.nodes[static_cast<std::size_t>(parent_ind)].children += 1;

  if (n.dist_to_goal < p_.delta_goal) res_.goal_nodes.push_back(new_ind);
  if (n.dist_to_goal < res_.nodes[static_cast<std::size_t>(res_.best_node)].dist_to_goal) res_.best_node = new_ind;
  return new_ind;
}

void SparseRrtBikePlanner::removeNode(int remove_ind) {
  // Port of MATLAB remove_Node (with recursive leaf removal and array shifts).
  const int n = static_cast<int>(res_.nodes.size());
  if (n - remove_ind < 3 || n < 2 || remove_ind == res_.best_node || remove_ind == 0) return;
  for (int gid : res_.goal_nodes) {
    if (gid == remove_ind) return;
  }

  // Recursively try to remove children first if they are leaves.
  while (true) {
    bool removed_child = false;
    for (int i = 0; i < static_cast<int>(res_.nodes.size()); ++i) {
      if (res_.nodes[static_cast<std::size_t>(i)].parent == remove_ind) {
        if (res_.nodes[static_cast<std::size_t>(i)].children == 0) {
          removeNode(i);
          removed_child = true;
          break;
        }
      }
    }
    if (!removed_child) break;
  }

  const int parent = res_.nodes[static_cast<std::size_t>(remove_ind)].parent;
  if (parent >= 0) res_.nodes[static_cast<std::size_t>(parent)].children -= 1;

  res_.nodes.erase(res_.nodes.begin() + remove_ind);

  // Fix parent indices greater than removed index
  for (auto& node : res_.nodes) {
    if (node.parent > remove_ind) node.parent -= 1;
  }

  // Fix goal_nodes indices
  for (auto& gid : res_.goal_nodes) {
    if (gid > remove_ind) gid -= 1;
  }

  if (res_.best_node > remove_ind) res_.best_node -= 1;
}

void SparseRrtBikePlanner::drain(int new_ind, int nearest_ind) {
  // MATLAB: dist in full 6D Euclidean, drain nodes within delta_drain excluding nearest_ind
  std::vector<int> drain_ids;
  for (int i = 0; i < static_cast<int>(res_.nodes.size()); ++i) {
    const auto& a = res_.nodes[static_cast<std::size_t>(i)].x;
    const auto& b = res_.nodes[static_cast<std::size_t>(new_ind)].x;
    double d2 = 0.0;
    for (int k = 0; k < 6; ++k) {
      const double dd = a[static_cast<std::size_t>(k)] - b[static_cast<std::size_t>(k)];
      d2 += dd * dd;
    }
    if (std::sqrt(d2) < p_.delta_drain && i != nearest_ind) drain_ids.push_back(i);
  }

  // Remove in reverse order to keep indices consistent-ish (MATLAB loops for i=1:L but remove shifts).
  std::sort(drain_ids.begin(), drain_ids.end());
  for (int idx = static_cast<int>(drain_ids.size()) - 1; idx >= 0; --idx) {
    const int i = drain_ids[static_cast<std::size_t>(idx)];
    if (i <= 0 || i >= static_cast<int>(res_.nodes.size())) continue;
    if (res_.nodes[static_cast<std::size_t>(i)].children == 0 &&
        res_.nodes[static_cast<std::size_t>(i)].cum_cost > res_.nodes[static_cast<std::size_t>(new_ind)].cum_cost) {
      res_.removed += 1;
      removeNode(i);
    } else if (res_.nodes[static_cast<std::size_t>(i)].cum_cost < res_.nodes[static_cast<std::size_t>(new_ind)].cum_cost) {
      res_.removed += 1;
      removeNode(new_ind);
      break;
    }
  }

  // RemovingHighCostNodes when goal reached and type=='p'
  if (!res_.goal_nodes.empty() && p_.type == 'p') {
    double best_goal_cost = std::numeric_limits<double>::infinity();
    for (int gid : res_.goal_nodes) best_goal_cost = std::min(best_goal_cost, res_.nodes[static_cast<std::size_t>(gid)].cum_cost);
    for (int i = static_cast<int>(res_.nodes.size()) - 1; i >= 0; --i) {
      if (res_.nodes[static_cast<std::size_t>(i)].cum_cost > best_goal_cost && res_.nodes[static_cast<std::size_t>(i)].children == 0) {
        removeNode(i);
      }
    }
  }
}

void SparseRrtBikePlanner::rewire(int /*new_ind*/, const std::vector<int>& /*neigh*/, int /*min_ind*/) {
  // Exact port of MATLAB rewire is implemented in a follow-up patch once we switch main to this planner.
  // (It needs closed-loop feasibility checks and subtree cost propagation.)
}

SparseRrtBikePlanner::ResultT SparseRrtBikePlanner::plan() {
  res_ = ResultT{};
  res_.nodes.reserve(static_cast<std::size_t>(p_.max_nodes));

  NodeT root{};
  root.x = start_;
  root.parent = -1;
  root.children = 0;
  root.cum_cost = 0.0;
  root.cum_time = 0.0;
  root.dist_to_goal = distToGoal(start_);
  res_.nodes.push_back(root);
  res_.best_node = 0;

  for (int it = 1; it <= p_.max_iterations; ++it) {
    if (static_cast<int>(res_.nodes.size()) >= p_.max_nodes) break;

    const double delta_near = p_.delta_near_base + (0.5 - uni_(rng_)); // MATLAB: 1 + (.5-rand)

    const auto rand_node = sample();
    const int nearest_ind = nearest(rand_node);
    const auto neigh = neighbors(rand_node, nearest_ind, delta_near);

    int min_ind = nearest_ind;
    if (p_.use_best_parent) {
      min_ind = findNewParent(neigh, rand_node, nearest_ind);
    }

    Vec<BikeModel::NX> new_state{};
    Vec<BikeModel::NU> u{};
    const bool feasible = steer(min_ind, rand_node, new_state, u);
    if (!feasible) continue;

    const int new_ind = addNode(min_ind, new_state, u);
    if (new_ind <= 0) continue;

    if (p_.use_rewire) rewire(new_ind, neigh, min_ind);
    if (p_.use_drain) drain(new_ind, min_ind);
  }

  return res_;
}

} // namespace ru_racer

