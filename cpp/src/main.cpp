#include "ru_racer/bike_model.hpp"
#include "ru_racer/config.hpp"
#include "ru_racer/csv.hpp"
#include "ru_racer/four_wheel_model.hpp"
#include "ru_racer/nmpc.hpp"
#include "ru_racer/occupancy_grid.hpp"
#include "ru_racer/rrt_star.hpp"
#include "ru_racer/sparse_rrt_bike.hpp"

#include "ru_racer/app/csv_dump.hpp"
#include "ru_racer/app/fs_utils.hpp"
#include "ru_racer/app/model_config.hpp"
#include "ru_racer/app/obstacles.hpp"
#include "ru_racer/app/validation.hpp"
#include "ru_racer/app/visualize.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace ru_racer;

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
  const fs::path out_dir = out_root / ("run_" + ru_racer::app::nowStamp());
  fs::create_directories(out_dir);

  const std::string mode = cfg.getString("mode", "plan");

  // Bike params
  BikeParams bp = ru_racer::app::loadBikeParamsFromConfig(cfg);
  BikeModel bike(bp);
  ru_racer::app::configureBikeRuntimeParams(bike, cfg);

  // Obstacles / map
  std::vector<CircleObstacle> circles;
  const auto obstacles_csv = cfg.getString("obstacles_csv", "");
  if (!obstacles_csv.empty()) {
    if (ru_racer::app::loadCircleObstaclesCsv(obstacles_csv, circles)) bike.setCircleObstacles(circles);
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

  // Validation mode: run open-loop scenarios in open space (no planning/tracking).
  if (mode == "validate") {
    const std::string models = cfg.getString("validate_models", "bike");
    bool ok = true;
    if (models == "bike" || models == "both") {
      ok = ok && ru_racer::app::runOpenLoopValidation(bike, cfg, out_dir);
    }
    if (models == "four_wheel" || models == "both") {
      FourWheelParams fp = ru_racer::app::loadFourWheelParamsFromConfig(cfg);
      FourWheelModel fw(fp);
      ru_racer::app::configureFourWheelRuntimeParams(fw, cfg);
      ok = ok && ru_racer::app::runOpenLoopValidationFourWheel(fw, cfg, out_dir);
    }
    // Copy config used
    {
      std::ifstream in(config_path);
      std::ofstream out(out_dir / "config_used.cfg");
      out << in.rdbuf();
    }
    std::cout << "Done (validate). Results in: " << out_dir << "\n";
    return ok ? 0 : 2;
  }

  // NMPC-only validation mode (generate feasible reference by simulation, then track).
  if (mode == "nmpc_validate") {
    const bool ok = ru_racer::app::runNmpcValidation(bike, cfg, out_dir);
    // Copy config used
    {
      std::ifstream in(config_path);
      std::ofstream out(out_dir / "config_used.cfg");
      out << in.rdbuf();
    }
    std::cout << "Done (nmpc_validate). Results in: " << out_dir << "\n";
    return ok ? 0 : 2;
  }

  auto writeConfigUsedWithOverrides = [&](const fs::path& out_path,
                                         const Vec<BikeModel::NX>& start_override,
                                         const Vec<BikeModel::NX>& goal_override,
                                         int seed_override) {
    std::ifstream in(config_path);
    std::ofstream out(out_path);
    out << in.rdbuf();
    out << "\n# ---- overrides (written by ru_racer) ----\n";
    out << "seed=" << seed_override << "\n";
    out << "start_x=" << start_override[0] << "\n";
    out << "start_y=" << start_override[1] << "\n";
    out << "start_psi=" << start_override[2] << "\n";
    out << "start_vx=" << start_override[3] << "\n";
    out << "start_vy=" << start_override[4] << "\n";
    out << "start_r=" << start_override[5] << "\n";
    out << "goal_x=" << goal_override[0] << "\n";
    out << "goal_y=" << goal_override[1] << "\n";
    out << "goal_psi=" << goal_override[2] << "\n";
    out << "goal_vx=" << goal_override[3] << "\n";
    out << "goal_vy=" << goal_override[4] << "\n";
    out << "goal_r=" << goal_override[5] << "\n";
  };

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

  // Core pipeline runner so we can reuse it for batch tests.
  auto runPlanAndTrackOnce = [&](const fs::path& this_out_dir,
                                 const Vec<BikeModel::NX>& this_start,
                                 const Vec<BikeModel::NX>& this_goal,
                                 int this_seed,
                                 bool auto_viz,
                                 double* out_best_cost,
                                 int* out_nodes) -> bool {
    BikeModel bike_local = bike;
    bike_local.setGoal(this_goal);

    std::vector<int> path_ids_local;
    std::vector<Vec<BikeModel::NX>> ref_local;

    if (!(algo == "spars_rrt" || algo == "spars_rrt_star" || algo == "rrt" || algo == "rrt_star")) {
      std::cerr << "Unknown algorithm=" << algo << " (expected rrt|rrt_star|spars_rrt|spars_rrt_star)\n";
      return false;
    }

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

    SparseRrtBikePlanner planner(bike_local, this_start, this_goal, sp);
    planner.seed(static_cast<std::uint64_t>(this_seed));

    std::cout << "Planning algorithm: " << algo << " ...\n";
    const auto res = planner.plan();
    if (out_nodes) *out_nodes = static_cast<int>(res.nodes.size());

    ru_racer::app::writeNodesCsvSparse(this_out_dir / "rrt_nodes.csv", res);

    int best_goal = -1;
    double best_cost = 1e100;
    for (int gid : res.goal_nodes) {
      const double c = res.nodes[static_cast<std::size_t>(gid)].cum_cost;
      if (c < best_cost) {
        best_cost = c;
        best_goal = gid;
      }
    }
    if (out_best_cost) *out_best_cost = best_cost;
    if (best_goal < 0) {
      std::cerr << "No goal reached. Wrote tree to: " << (this_out_dir / "rrt_nodes.csv") << "\n";
      return false;
    }

    path_ids_local = ru_racer::app::extractPathSparse(res, best_goal);
    ru_racer::app::writePathCsvSparse(this_out_dir / "rrt_path.csv", path_ids_local, res);

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
    ref_local = ru_racer::app::buildReferenceTrajectory(bike_local, path_ids_local, fake, rp.max_step);

    {
      CsvWriter w(this_out_dir / "rrt_ref_traj.csv");
      w.writeRow("k", "t", "x", "y", "psi", "vx", "vy", "r");
      for (std::size_t k = 0; k < ref_local.size(); ++k) {
        const auto& x = ref_local[k];
        w.writeRow(static_cast<int>(k), k * bike_local.dt(), x[0], x[1], x[2], x[3], x[4], x[5]);
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

      NmpcTracker tracker(bike_local, np);
      std::cout << "Tracking with NMPC...\n";
      const auto exec = tracker.track(ref_local);

      CsvWriter w(this_out_dir / "nmpc_exec.csv");
      w.writeRow("k", "t", "x", "y", "psi", "vx", "vy", "r", "u_delta", "u_sfx", "x_ref", "y_ref", "psi_ref", "vx_ref", "vy_ref", "r_ref");
      for (std::size_t k = 0; k < exec.size(); ++k) {
        const auto& s = exec[k];
        w.writeRow(static_cast<int>(k), s.t,
                   s.x[0], s.x[1], s.x[2], s.x[3], s.x[4], s.x[5],
                   s.u[0], s.u[1],
                   s.x_ref[0], s.x_ref[1], s.x_ref[2], s.x_ref[3], s.x_ref[4], s.x_ref[5]);
      }
    }

    writeConfigUsedWithOverrides(this_out_dir / "config_used.cfg", this_start, this_goal, this_seed);

    if (auto_viz) {
      const fs::path bg = cfg.getString("viz_bg", "");
      const std::string bg_transform = cfg.getString("viz_bg_transform", "none");
      const bool show_start_goal = cfg.getBool("viz_show_start_goal", true);
      ru_racer::app::tryAutoVisualize(this_out_dir, bg, bg_transform, show_start_goal);
    }

    return true;
  };

  // Batch mode: sample multiple random goal points in an open space box and run planner+NMPC for each.
  if (mode == "batch_setpoints") {
    const int count = std::max(1, cfg.getInt("batch_count", 10));
    const int base_seed = cfg.getInt("seed", 1234);
    const double gxmin = cfg.getDouble("batch_goal_xmin", -4.0);
    const double gxmax = cfg.getDouble("batch_goal_xmax", 4.0);
    const double gymin = cfg.getDouble("batch_goal_ymin", -4.0);
    const double gymax = cfg.getDouble("batch_goal_ymax", 4.0);
    const double gmin_dist = cfg.getDouble("batch_goal_min_dist", 0.5);
    const double goal_vx = cfg.getDouble("batch_goal_vx", goal[3]);
    const double goal_psi = cfg.getDouble("batch_goal_psi", goal[2]);

    const bool per_goal_viz = cfg.getBool("batch_per_goal_visualize", false);

    std::mt19937_64 rng(static_cast<std::uint64_t>(base_seed));
    std::uniform_real_distribution<double> dx(gxmin, gxmax);
    std::uniform_real_distribution<double> dy(gymin, gymax);

    CsvWriter sum(out_dir / "batch_summary.csv");
    sum.writeRow("i", "seed", "goal_x", "goal_y", "success", "best_cost", "nodes");

    int successes = 0;
    for (int i = 0; i < count; ++i) {
      Vec<BikeModel::NX> g = goal;
      // sample until not too close to start
      for (int tries = 0; tries < 1000; ++tries) {
        const double gx = dx(rng);
        const double gy = dy(rng);
        const double dist = std::hypot(gx - start[0], gy - start[1]);
        if (dist >= gmin_dist) {
          g[0] = gx;
          g[1] = gy;
          break;
        }
      }
      g[2] = goal_psi;
      g[3] = goal_vx;
      g[4] = 0.0;
      g[5] = 0.0;

      std::ostringstream name;
      name << "goal_" << std::setw(3) << std::setfill('0') << i;
      const fs::path sub = out_dir / name.str();
      fs::create_directories(sub);

      const int seed_i = base_seed + i;
      double best_cost = 0.0;
      int nodes = 0;
      std::cout << "\n=== Batch goal " << i << "/" << count << " : (" << g[0] << ", " << g[1] << ") ===\n";
      const bool ok = runPlanAndTrackOnce(sub, start, g, seed_i, per_goal_viz, &best_cost, &nodes);
      sum.writeRow(i, seed_i, g[0], g[1], ok ? 1 : 0, best_cost, nodes);
      if (ok) successes += 1;
    }

    // Copy config used at root
    {
      std::ifstream in(config_path);
      std::ofstream out(out_dir / "config_used.cfg");
      out << in.rdbuf();
    }

    std::cout << "Done (batch_setpoints). Successes: " << successes << "/" << count << ". Results in: " << out_dir << "\n";
    return successes > 0 ? 0 : 1;
  }

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

    ru_racer::app::writeNodesCsvSparse(out_dir / "rrt_nodes.csv", res);

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

    path_ids = ru_racer::app::extractPathSparse(res, best_goal);
    ru_racer::app::writePathCsvSparse(out_dir / "rrt_path.csv", path_ids, res);

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
    ref = ru_racer::app::buildReferenceTrajectory(bike, path_ids, fake, rp.max_step);
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

  // Auto-generate visuals
  const bool auto_viz = cfg.getBool("auto_visualize", true);
  if (auto_viz) {
    const fs::path bg = cfg.getString("viz_bg", "track.png");
    const std::string bg_transform = cfg.getString("viz_bg_transform", "track");
    const bool show_start_goal = cfg.getBool("viz_show_start_goal", true);
    ru_racer::app::tryAutoVisualize(out_dir, bg, bg_transform, show_start_goal);
  }

  std::cout << "Done. Results in: " << out_dir << "\n";
  return 0;
}

