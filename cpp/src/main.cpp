#include "ru_racer/bike_model.hpp"
#include "ru_racer/config.hpp"
#include "ru_racer/csv.hpp"
#include "ru_racer/nmpc.hpp"
#include "ru_racer/occupancy_grid.hpp"
#include "ru_racer/rrt_star.hpp"
#include "ru_racer/sparse_rrt_bike.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace ru_racer;

static std::string nowStamp() {
  using clock = std::chrono::system_clock;
  const auto t = clock::to_time_t(clock::now());
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
                tm.tm_min, tm.tm_sec);
  return std::string(buf);
}

static bool loadCircleObstaclesCsv(const fs::path& path, std::vector<CircleObstacle>& out) {
  std::ifstream in(path);
  if (!in.good()) return false;
  out.clear();
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::stringstream ss(line);
    std::string a, b, c;
    if (!std::getline(ss, a, ',')) continue;
    if (!std::getline(ss, b, ',')) continue;
    if (!std::getline(ss, c, ',')) continue;
    CircleObstacle o{};
    o.x = std::stod(a);
    o.y = std::stod(b);
    o.r = std::stod(c);
    out.push_back(o);
  }
  return true;
}

static void writePathCsv(const fs::path& path,
                         const std::vector<int>& path_ids,
                         const std::vector<RrtNode>& nodes) {
  CsvWriter w(path);
  w.writeRow("idx", "node_id", "parent", "cum_cost", "cum_time", "x", "y", "psi", "vx", "vy", "r", "u_delta", "u_sfx");
  for (std::size_t i = 0; i < path_ids.size(); ++i) {
    const int id = path_ids[i];
    const auto& n = nodes[static_cast<std::size_t>(id)];
    w.writeRow(static_cast<int>(i), id, n.parent, n.cum_cost, n.cum_time, n.x[0], n.x[1], n.x[2], n.x[3], n.x[4], n.x[5],
               n.u_from_parent[0], n.u_from_parent[1]);
  }
}

static void writeNodesCsv(const fs::path& path, const std::vector<RrtNode>& nodes) {
  CsvWriter w(path);
  w.writeRow("node_id", "parent", "cost_from_parent", "cum_cost", "time_from_parent", "cum_time", "dist_to_goal",
             "x", "y", "psi", "vx", "vy", "r", "u_delta", "u_sfx");
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const auto& n = nodes[i];
    w.writeRow(static_cast<int>(i), n.parent, n.cost_from_parent, n.cum_cost, n.time_from_parent, n.cum_time, n.dist_to_goal,
               n.x[0], n.x[1], n.x[2], n.x[3], n.x[4], n.x[5], n.u_from_parent[0], n.u_from_parent[1]);
  }
}

static std::vector<Vec<BikeModel::NX>> buildReferenceTrajectory(const BikeModel& model_template,
                                                                const std::vector<int>& path_ids,
                                                                const std::vector<RrtNode>& nodes,
                                                                double segment_max_step) {
  std::vector<Vec<BikeModel::NX>> ref;
  if (path_ids.empty()) return ref;

  BikeModel m = model_template;
  m.reset(nodes[static_cast<std::size_t>(path_ids.front())].x, Vec<BikeModel::NU>{0.0, 0.0});
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

static void writeNodesCsvSparse(const fs::path& path, const SparseRrtBikePlanner::ResultT& res) {
  CsvWriter w(path);
  w.writeRow("node_id", "parent", "children", "success", "cost_from_parent", "cum_cost", "time_from_parent", "cum_time", "dist_to_goal",
             "x", "y", "psi", "vx", "vy", "r", "u_delta", "u_sfx");
  for (std::size_t i = 0; i < res.nodes.size(); ++i) {
    const auto& n = res.nodes[i];
    w.writeRow(static_cast<int>(i), n.parent, n.children, n.success,
               n.cost_from_parent, n.cum_cost, n.time_from_parent, n.cum_time, n.dist_to_goal,
               n.x[0], n.x[1], n.x[2], n.x[3], n.x[4], n.x[5],
               n.u_from_parent[0], n.u_from_parent[1]);
  }
}

static void writePathCsvSparse(const fs::path& path, const std::vector<int>& path_ids, const SparseRrtBikePlanner::ResultT& res) {
  CsvWriter w(path);
  w.writeRow("idx", "node_id", "parent", "cum_cost", "cum_time", "x", "y", "psi", "vx", "vy", "r", "u_delta", "u_sfx");
  for (std::size_t i = 0; i < path_ids.size(); ++i) {
    const int id = path_ids[i];
    const auto& n = res.nodes[static_cast<std::size_t>(id)];
    w.writeRow(static_cast<int>(i), id, n.parent, n.cum_cost, n.cum_time, n.x[0], n.x[1], n.x[2], n.x[3], n.x[4], n.x[5],
               n.u_from_parent[0], n.u_from_parent[1]);
  }
}

static std::vector<int> extractPathSparse(const SparseRrtBikePlanner::ResultT& res, int target) {
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

int main(int argc, char** argv) {
  fs::path config_path = fs::path("cpp") / "data" / "default.cfg";
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    }
  }

  KeyValueConfig cfg;
  if (!cfg.load(config_path)) {
    std::cerr << "Failed to load config: " << config_path << "\n";
    return 2;
  }

  const fs::path out_root = cfg.getString("output_root", "cpp/results");
  const fs::path out_dir = out_root / ("run_" + nowStamp());
  fs::create_directories(out_dir);

  // Bike params
  BikeParams bp{};
  bp.dt = cfg.getDouble("dt", bp.dt);
  bp.rewiring_value = cfg.getDouble("rewiring_value", bp.rewiring_value);

  BikeModel bike(bp);

  // Obstacles / map
  std::vector<CircleObstacle> circles;
  const auto obstacles_csv = cfg.getString("obstacles_csv", "");
  if (!obstacles_csv.empty()) {
    if (loadCircleObstaclesCsv(obstacles_csv, circles)) bike.setCircleObstacles(circles);
  }

  OccupancyGrid grid;
  const auto map_file = cfg.getString("map_file", "");
  const auto map_pgm = cfg.getString("map_pgm", ""); // legacy
  const auto map_path = !map_file.empty() ? map_file : map_pgm;
  if (!map_path.empty()) {
    std::string err;
    grid.invert = cfg.getBool("map_invert", grid.invert);
    grid.free_threshold = static_cast<std::uint8_t>(cfg.getInt("map_free_threshold", static_cast<int>(grid.free_threshold)));

    // Optional affine transform (preferred when calibrating to an image map)
    const bool use_affine = cfg.getBool("map_use_affine", false);
    grid.use_affine_transform = use_affine;
    if (use_affine) {
      grid.a11 = cfg.getDouble("map_a11", 0.0);
      grid.a12 = cfg.getDouble("map_a12", 0.0);
      grid.a21 = cfg.getDouble("map_a21", 0.0);
      grid.a22 = cfg.getDouble("map_a22", 0.0);
      grid.b1 = cfg.getDouble("map_b1", 0.0);
      grid.b2 = cfg.getDouble("map_b2", 0.0);
    } else {
      grid.pixels_per_meter = cfg.getDouble("map_pixels_per_meter", grid.pixels_per_meter);
      grid.origin_x_px = cfg.getDouble("map_origin_x_px", grid.origin_x_px);
      grid.origin_y_px = cfg.getDouble("map_origin_y_px", grid.origin_y_px);
    }

    if (grid.load(map_path, &err)) {
      bike.setOccupancyGrid(&grid);
    } else {
      std::cerr << "WARN: failed to load map file: " << err << "\n";
    }
  } else {
    std::cerr << "WARN: no occupancy map configured; planner will ignore track boundaries.\n";
  }

  // Start/goal
  Vec<BikeModel::NX> start{cfg.getDouble("start_x", 0.0),
                           cfg.getDouble("start_y", 0.01),
                           cfg.getDouble("start_psi", M_PI / 2.0),
                           cfg.getDouble("start_vx", 3.0),
                           cfg.getDouble("start_vy", 0.0),
                           cfg.getDouble("start_r", 0.0)};
  Vec<BikeModel::NX> goal{cfg.getDouble("goal_x", 0.0),
                          cfg.getDouble("goal_y", -0.05),
                          cfg.getDouble("goal_psi", 2.0 * M_PI + M_PI / 2.0),
                          cfg.getDouble("goal_vx", 1.0),
                          cfg.getDouble("goal_vy", 0.0),
                          cfg.getDouble("goal_r", 0.0)};
  bike.setGoal(goal);

  // RRT* params
  RrtStarParams rp{};
  rp.max_nodes = cfg.getInt("max_nodes", rp.max_nodes);
  rp.max_iterations = cfg.getInt("max_iterations", rp.max_iterations);
  rp.delta_goal = cfg.getDouble("delta_goal", rp.delta_goal);
  rp.max_step = cfg.getDouble("max_step", rp.max_step);
  rp.goal_sample_rate = cfg.getDouble("goal_sample_rate", rp.goal_sample_rate);
  rp.neighbor_gamma = cfg.getDouble("neighbor_gamma", rp.neighbor_gamma);
  rp.neighbor_r_max = cfg.getDouble("neighbor_r_max", rp.neighbor_r_max);
  rp.delta_max = cfg.getDouble("delta_max", rp.delta_max);
  rp.sfx_max = cfg.getDouble("sfx_max", rp.sfx_max);

  rp.x_min = Vec<BikeModel::NX>{cfg.getDouble("xmin_x", rp.x_min[0]), cfg.getDouble("xmin_y", rp.x_min[1]), cfg.getDouble("xmin_psi", rp.x_min[2]),
                                cfg.getDouble("xmin_vx", rp.x_min[3]), cfg.getDouble("xmin_vy", rp.x_min[4]), cfg.getDouble("xmin_r", rp.x_min[5])};
  rp.x_max = Vec<BikeModel::NX>{cfg.getDouble("xmax_x", rp.x_max[0]), cfg.getDouble("xmax_y", rp.x_max[1]), cfg.getDouble("xmax_psi", rp.x_max[2]),
                                cfg.getDouble("xmax_vx", rp.x_max[3]), cfg.getDouble("xmax_vy", rp.x_max[4]), cfg.getDouble("xmax_r", rp.x_max[5])};

  const std::string algo = cfg.getString("algorithm", "spars_rrt");

  std::vector<int> path_ids;
  std::vector<Vec<BikeModel::NX>> ref;

  if (algo == "spars_rrt" || algo == "spars_rrt_star" || algo == "rrt" || algo == "rrt_star") {
    SparseRrtBikeParams sp{};
    sp.max_iterations = rp.max_iterations;
    sp.max_nodes = rp.max_nodes;
    sp.delta_goal = rp.delta_goal;
    sp.delta_drain = cfg.getDouble("delta_drain", sp.delta_drain);
    sp.max_step = rp.max_step;
    sp.delta_max = rp.delta_max;
    sp.sfx_max = rp.sfx_max;
    sp.x_min = rp.x_min;
    sp.x_max = rp.x_max;
    sp.type = cfg.getString("type", "o").empty() ? 'o' : cfg.getString("type", "o")[0];

    // Map algorithm to switches (so we can compare fairly).
    if (algo == "rrt") {
      sp.use_best_parent = false;
      sp.use_rewire = false;
      sp.use_drain = false;
    } else if (algo == "rrt_star") {
      sp.use_best_parent = true;
      sp.use_rewire = true;
      sp.use_drain = false;
    } else if (algo == "spars_rrt") {
      sp.use_best_parent = true;
      sp.use_rewire = false;
      sp.use_drain = true;
    } else { // spars_rrt_star
      sp.use_best_parent = true;
      sp.use_rewire = true;
      sp.use_drain = true;
    }

    SparseRrtBikePlanner planner(bike, start, goal, sp);
    planner.seed(static_cast<std::uint64_t>(cfg.getInt("seed", 1234)));

    std::cout << "Planning algorithm: " << algo << " ...\n";
    const auto res = planner.plan();

    writeNodesCsvSparse(out_dir / "rrt_nodes.csv", res);

    // Pick best goal node (min cum_cost), like MATLAB plotting.
    int best_goal = -1;
    double best_cost = 1e100;
    for (int gid : res.goal_nodes) {
      const double c = res.nodes[static_cast<std::size_t>(gid)].cum_cost;
      if (c < best_cost) {
        best_cost = c;
        best_goal = gid;
      }
    }
    if (best_goal < 0) {
      std::cerr << "No goal reached. Wrote tree to: " << (out_dir / "rrt_nodes.csv") << "\n";
      return 1;
    }

    path_ids = extractPathSparse(res, best_goal);
    writePathCsvSparse(out_dir / "rrt_path.csv", path_ids, res);

    // Convert sparse nodes format into dt-reference by replaying u_from_parent.
    // This is analogous to MATLAB FollowTrajectory.
    // We'll reuse the same helper by adapting to RrtNode-like access:
    // simplest: create fake RrtNode list.
    std::vector<RrtNode> fake;
    fake.reserve(res.nodes.size());
    for (const auto& n : res.nodes) {
      RrtNode rn{};
      rn.x = n.x;
      rn.u_from_parent = n.u_from_parent;
      rn.parent = n.parent;
      rn.cum_cost = n.cum_cost;
      rn.cum_time = n.cum_time;
      rn.time_from_parent = n.time_from_parent;
      fake.push_back(rn);
    }
    ref = buildReferenceTrajectory(bike, path_ids, fake, rp.max_step);
  } else {
    std::cerr << "Unknown algorithm=" << algo << " (expected rrt|rrt_star|spars_rrt|spars_rrt_star)\n";
    return 2;
  }
  {
    CsvWriter w(out_dir / "rrt_ref_traj.csv");
    w.writeRow("k", "t", "x", "y", "psi", "vx", "vy", "r");
    for (std::size_t k = 0; k < ref.size(); ++k) {
      const auto& x = ref[k];
      w.writeRow(static_cast<int>(k), k * bike.dt(), x[0], x[1], x[2], x[3], x[4], x[5]);
    }
  }

  const bool run_nmpc = cfg.getBool("run_nmpc", true);
  if (run_nmpc) {
    NmpcParams np{};
    np.horizon_steps = cfg.getInt("nmpc_horizon_steps", np.horizon_steps);
    np.max_iters = cfg.getInt("nmpc_max_iters", np.max_iters);
    np.step_size = cfg.getDouble("nmpc_step_size", np.step_size);
    np.eps_fd = cfg.getDouble("nmpc_eps_fd", np.eps_fd);
    np.control_hold_steps = cfg.getInt("nmpc_control_hold_steps", np.control_hold_steps);
    np.w_pos = cfg.getDouble("nmpc_w_pos", np.w_pos);
    np.w_psi = cfg.getDouble("nmpc_w_psi", np.w_psi);
    np.w_vx = cfg.getDouble("nmpc_w_vx", np.w_vx);
    np.w_u = cfg.getDouble("nmpc_w_u", np.w_u);
    np.delta_max = rp.delta_max;
    np.sfx_max = rp.sfx_max;

    NmpcTracker tracker(bike, np);
    std::cout << "Tracking with NMPC...\n";
    const auto exec = tracker.track(ref);

    CsvWriter w(out_dir / "nmpc_exec.csv");
    w.writeRow("k", "t", "x", "y", "psi", "vx", "vy", "r", "u_delta", "u_sfx", "x_ref", "y_ref", "psi_ref", "vx_ref", "vy_ref", "r_ref");
    for (std::size_t k = 0; k < exec.size(); ++k) {
      const auto& s = exec[k];
      w.writeRow(static_cast<int>(k), s.t,
                 s.x[0], s.x[1], s.x[2], s.x[3], s.x[4], s.x[5],
                 s.u[0], s.u[1],
                 s.x_ref[0], s.x_ref[1], s.x_ref[2], s.x_ref[3], s.x_ref[4], s.x_ref[5]);
    }
  }

  // Copy config used
  {
    std::ifstream in(config_path);
    std::ofstream out(out_dir / "config_used.cfg");
    out << in.rdbuf();
  }

  std::cout << "Done. Results in: " << out_dir << "\n";
  return 0;
}

