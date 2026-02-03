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
#include <iostream>
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

