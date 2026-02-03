#pragma once

#include "ru_racer/bike_model.hpp"
#include "ru_racer/csv.hpp"
#include "ru_racer/planner_types.hpp"
#include "ru_racer/rrt_star.hpp"
#include "ru_racer/sparse_rrt_bike.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <vector>

namespace ru_racer::app {

inline void writePathCsv(const std::filesystem::path& path,
                         const std::vector<int>& path_ids,
                         const std::vector<ru_racer::RrtNode>& nodes) {
  ru_racer::CsvWriter w(path);
  w.writeRow("idx", "node_id", "parent", "cum_cost", "cum_time", "x", "y", "psi", "vx", "vy", "r", "u_delta", "u_sfx");
  for (std::size_t i = 0; i < path_ids.size(); ++i) {
    const int id = path_ids[i];
    const auto& n = nodes[static_cast<std::size_t>(id)];
    w.writeRow(static_cast<int>(i), id, n.parent, n.cum_cost, n.cum_time, n.x[0], n.x[1], n.x[2], n.x[3], n.x[4], n.x[5], n.u_from_parent[0],
               n.u_from_parent[1]);
  }
}

inline void writeNodesCsv(const std::filesystem::path& path, const std::vector<ru_racer::RrtNode>& nodes) {
  ru_racer::CsvWriter w(path);
  w.writeRow("node_id", "parent", "cost_from_parent", "cum_cost", "time_from_parent", "cum_time", "dist_to_goal", "x", "y", "psi", "vx", "vy", "r",
             "u_delta", "u_sfx");
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const auto& n = nodes[i];
    w.writeRow(static_cast<int>(i), n.parent, n.cost_from_parent, n.cum_cost, n.time_from_parent, n.cum_time, n.dist_to_goal, n.x[0], n.x[1], n.x[2],
               n.x[3], n.x[4], n.x[5], n.u_from_parent[0], n.u_from_parent[1]);
  }
}

inline std::vector<ru_racer::Vec<ru_racer::BikeModel::NX>> buildReferenceTrajectory(const ru_racer::BikeModel& model_template,
                                                                                   const std::vector<int>& path_ids,
                                                                                   const std::vector<ru_racer::RrtNode>& nodes,
                                                                                   double segment_max_step) {
  std::vector<ru_racer::Vec<ru_racer::BikeModel::NX>> ref;
  if (path_ids.empty()) return ref;

  ru_racer::BikeModel m = model_template;
  m.reset(nodes[static_cast<std::size_t>(path_ids.front())].x, ru_racer::Vec<ru_racer::BikeModel::NU>{0.0, 0.0});
  ref.push_back(m.state());

  for (std::size_t i = 1; i < path_ids.size(); ++i) {
    const auto& child = nodes[static_cast<std::size_t>(path_ids[i])];
    const auto u = child.u_from_parent;
    const double dur = (child.time_from_parent > 0.0 && child.time_from_parent < segment_max_step) ? child.time_from_parent : segment_max_step;
    const int steps = std::max(1, static_cast<int>(std::floor(dur / m.dt() + 1e-9)));
    for (int k = 0; k < steps; ++k) {
      m.step(u);
      ref.push_back(m.state());
    }
  }
  return ref;
}

inline void writeNodesCsvSparse(const std::filesystem::path& path, const ru_racer::SparseRrtBikePlanner::ResultT& res) {
  ru_racer::CsvWriter w(path);
  w.writeRow("node_id", "parent", "children", "success", "cost_from_parent", "cum_cost", "time_from_parent", "cum_time", "dist_to_goal", "x", "y",
             "psi", "vx", "vy", "r", "u_delta", "u_sfx");
  for (std::size_t i = 0; i < res.nodes.size(); ++i) {
    const auto& n = res.nodes[i];
    w.writeRow(static_cast<int>(i), n.parent, n.children, n.success, n.cost_from_parent, n.cum_cost, n.time_from_parent, n.cum_time, n.dist_to_goal, n.x[0],
               n.x[1], n.x[2], n.x[3], n.x[4], n.x[5], n.u_from_parent[0], n.u_from_parent[1]);
  }
}

inline void writePathCsvSparse(const std::filesystem::path& path, const std::vector<int>& path_ids, const ru_racer::SparseRrtBikePlanner::ResultT& res) {
  ru_racer::CsvWriter w(path);
  w.writeRow("idx", "node_id", "parent", "cum_cost", "cum_time", "x", "y", "psi", "vx", "vy", "r", "u_delta", "u_sfx");
  for (std::size_t i = 0; i < path_ids.size(); ++i) {
    const int id = path_ids[i];
    const auto& n = res.nodes[static_cast<std::size_t>(id)];
    w.writeRow(static_cast<int>(i), id, n.parent, n.cum_cost, n.cum_time, n.x[0], n.x[1], n.x[2], n.x[3], n.x[4], n.x[5], n.u_from_parent[0],
               n.u_from_parent[1]);
  }
}

inline std::vector<int> extractPathSparse(const ru_racer::SparseRrtBikePlanner::ResultT& res, int target) {
  if (target < 0 || target >= static_cast<int>(res.nodes.size())) return {};
  std::vector<int> p;
  int cur = target;
  while (cur >= 0) {
    p.push_back(cur);
    cur = res.nodes[static_cast<std::size_t>(cur)].parent;
  }
  std::reverse(p.begin(), p.end());
  return p;
}

} // namespace ru_racer::app

