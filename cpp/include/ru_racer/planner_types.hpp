#pragma once

#include "ru_racer/math.hpp"

#include <cstddef>
#include <vector>

namespace ru_racer {

template <std::size_t NX, std::size_t NU>
struct PlannerNode {
  Vec<NX> x{};
  Vec<NU> u_from_parent{};
  int parent = -1;
  int children = 0;
  double time_from_parent = 0.0;
  double cum_time = 0.0;
  double cost_from_parent = 0.0;
  double cum_cost = 0.0;
  double dist_to_goal = 0.0;
  int success = 0;
};

template <std::size_t NX, std::size_t NU>
struct PlannerResult {
  std::vector<PlannerNode<NX, NU>> nodes;
  std::vector<int> goal_nodes;
  int best_node = 0;
  int removed = 0;
  int rewired = 0;
};

} // namespace ru_racer

