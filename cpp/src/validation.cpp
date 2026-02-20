#include "ru_racer/app/validation.hpp"

#include "ru_racer/app/fs_utils.hpp"
#include "ru_racer/four_wheel_model.hpp"
#include "ru_racer/csv.hpp"
#include "ru_racer/nmpc.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <cctype>
#include <vector>

namespace fs = std::filesystem;

namespace ru_racer::app {
namespace {

constexpr double deg2rad(double d) { return d * M_PI / 180.0; }

struct Scenario {
  std::string name;
  double duration_s = 5.0;
  std::function<Vec<BikeModel::NU>(double t, const Vec<BikeModel::NX>& x)> control;
  std::function<void(BikeModel& bike, double t)> friction_update; // optional
};

std::vector<std::string> splitCsvList(const std::string& s) {
  std::vector<std::string> out;
  std::string cur;
  std::istringstream iss(s);
  while (std::getline(iss, cur, ',')) {
    // trim
    const auto b = cur.find_first_not_of(" \t\r\n");
    const auto e = cur.find_last_not_of(" \t\r\n");
    if (b == std::string::npos) continue;
    out.push_back(cur.substr(b, e - b + 1));
  }
  return out;
}

struct NmpcValScenario {
  std::string name;
  double duration_s = 8.0;
  std::function<Vec<BikeModel::NU>(double t, const Vec<BikeModel::NX>& x)> u_ref;
};

} // namespace

bool runOpenLoopValidation(BikeModel& bike, const KeyValueConfig& cfg, const fs::path& out_dir) {
  const fs::path vdir = out_dir / "validation" / "bike";
  fs::create_directories(vdir);

  // Base initial state and input
  const Vec<BikeModel::NX> x0{cfg.getDouble("start_x", 0.0),
                              cfg.getDouble("start_y", 0.0),
                              cfg.getDouble("start_psi", 0.0),
                              cfg.getDouble("start_vx", 10.0),
                              cfg.getDouble("start_vy", 0.0),
                              cfg.getDouble("start_r", 0.0)};
  const Vec<BikeModel::NU> u0{0.0, 0.0};

  // Open-space bounds (keep very wide)
  Vec<BikeModel::NX> xmin{}, xmax{};
  xmin[0] = cfg.getDouble("validate_xmin_x", -1e6);
  xmin[1] = cfg.getDouble("validate_xmin_y", -1e6);
  xmin[2] = cfg.getDouble("validate_xmin_psi", -1e6);
  xmin[3] = cfg.getDouble("validate_xmin_vx", -1e6);
  xmin[4] = cfg.getDouble("validate_xmin_vy", -1e6);
  xmin[5] = cfg.getDouble("validate_xmin_r", -1e6);
  xmax[0] = cfg.getDouble("validate_xmax_x", 1e6);
  xmax[1] = cfg.getDouble("validate_xmax_y", 1e6);
  xmax[2] = cfg.getDouble("validate_xmax_psi", 1e6);
  xmax[3] = cfg.getDouble("validate_xmax_vx", 1e6);
  xmax[4] = cfg.getDouble("validate_xmax_vy", 1e6);
  xmax[5] = cfg.getDouble("validate_xmax_r", 1e6);
  bike.setBounds(xmin, xmax);

  const double delta_max = cfg.getDouble("delta_max", deg2rad(30.0));
  const double sfx_max = cfg.getDouble("sfx_max", 0.05);

  const double accel_sfx = cfg.getDouble("validate_accel_sfx", 0.8 * sfx_max);
  const double brake_sfx = cfg.getDouble("validate_brake_sfx", -0.8 * sfx_max);
  const double steer_const = cfg.getDouble("validate_steer_const_rad", deg2rad(10.0));
  const double steer_amp = cfg.getDouble("validate_steer_sine_amp_rad", deg2rad(8.0));
  const double steer_freq = cfg.getDouble("validate_steer_sine_hz", 0.25);
  const double cruise_sfx = cfg.getDouble("validate_cruise_sfx", 0.2 * sfx_max);

  // Optional: cut longitudinal command to zero after some time for selected scenarios (helps isolate lateral dynamics).
  const double lon_cutoff_s = cfg.getDouble("validate_lon_cutoff_s", -1.0);
  const auto lon_cutoff_scenarios = splitCsvList(cfg.getString("validate_lon_cutoff_scenarios", "turn_sine_steer,mu_drop_sine_steer"));
  auto useLonCutoff = [&](const std::string& sc_name, double t) {
    if (lon_cutoff_s < 0.0) return false;
    if (std::find(lon_cutoff_scenarios.begin(), lon_cutoff_scenarios.end(), sc_name) == lon_cutoff_scenarios.end()) return false;
    return t >= lon_cutoff_s;
  };

  const double t_accel = cfg.getDouble("validate_t_accel", 6.0);
  const double t_brake = cfg.getDouble("validate_t_brake", 4.0);
  const double t_turn = cfg.getDouble("validate_t_turn", 8.0);
  const double t_sine = cfg.getDouble("validate_t_sine", 10.0);
  const double t_mu_drop = cfg.getDouble("validate_t_mu_drop", 10.0);

  const double mu_f_start = cfg.getDouble("validate_mu_f_start", bike.muScaleFront());
  const double mu_r_start = cfg.getDouble("validate_mu_r_start", bike.muScaleRear());
  const double mu_f_end = cfg.getDouble("validate_mu_f_end", 0.6);
  const double mu_r_end = cfg.getDouble("validate_mu_r_end", 0.6);

  // Define scenarios (open-loop)
  std::vector<Scenario> all;
  all.push_back(Scenario{
      "accel_straight",
      t_accel,
      [=](double /*t*/, const Vec<BikeModel::NX>& /*x*/) {
        return Vec<BikeModel::NU>{0.0, std::clamp(accel_sfx, -sfx_max, sfx_max)};
      },
      {}});

  all.push_back(Scenario{
      "brake_straight",
      t_brake,
      [=](double /*t*/, const Vec<BikeModel::NX>& /*x*/) {
        return Vec<BikeModel::NU>{0.0, std::clamp(brake_sfx, -sfx_max, sfx_max)};
      },
      {}});

  all.push_back(Scenario{
      "turn_constant_steer",
      t_turn,
      [=](double /*t*/, const Vec<BikeModel::NX>& /*x*/) {
        return Vec<BikeModel::NU>{std::clamp(steer_const, -delta_max, delta_max), std::clamp(cruise_sfx, -sfx_max, sfx_max)};
      },
      {}});

  all.push_back(Scenario{
      "turn_sine_steer",
      t_sine,
      [=](double t, const Vec<BikeModel::NX>& /*x*/) {
        const double delta = steer_amp * std::sin(2.0 * M_PI * steer_freq * t);
        const double sfx = useLonCutoff("turn_sine_steer", t) ? 0.0 : cruise_sfx;
        return Vec<BikeModel::NU>{std::clamp(delta, -delta_max, delta_max), std::clamp(sfx, -sfx_max, sfx_max)};
      },
      {}});

  all.push_back(Scenario{
      "mu_drop_sine_steer",
      t_mu_drop,
      [=](double t, const Vec<BikeModel::NX>& /*x*/) {
        const double delta = steer_amp * std::sin(2.0 * M_PI * steer_freq * t);
        const double sfx = useLonCutoff("mu_drop_sine_steer", t) ? 0.0 : cruise_sfx;
        return Vec<BikeModel::NU>{std::clamp(delta, -delta_max, delta_max), std::clamp(sfx, -sfx_max, sfx_max)};
      },
      [=](BikeModel& b, double t) {
        const double a = (t_mu_drop <= 0.0) ? 1.0 : std::clamp(t / t_mu_drop, 0.0, 1.0);
        const double muf = (1.0 - a) * mu_f_start + a * mu_f_end;
        const double mur = (1.0 - a) * mu_r_start + a * mu_r_end;
        b.setFrictionScales(muf, mur);
      }});

  // Choose which to run
  const std::string list = cfg.getString("validate_scenarios", "accel_straight,brake_straight,turn_constant_steer,turn_sine_steer,mu_drop_sine_steer");
  const auto wanted = splitCsvList(list);
  auto wantIt = [&](const std::string& name) {
    if (wanted.empty()) return true;
    return std::find(wanted.begin(), wanted.end(), name) != wanted.end();
  };

  bool ran_any = false;
  for (const auto& sc : all) {
    if (!wantIt(sc.name)) continue;
    ran_any = true;

    // Reset bike and friction each scenario
    bike.reset(x0, u0);
    bike.setFrictionScales(mu_f_start, mu_r_start);

    const int steps = std::max(1, static_cast<int>(std::floor(sc.duration_s / bike.dt() + 1e-9)));
    const fs::path csv_path = vdir / (sc.name + ".csv");
    CsvWriter w(csv_path);
    w.writeRow("scenario", "k", "t", "x", "y", "psi", "vx", "vy", "r", "u_delta", "u_sfx", "mu_scale_f", "mu_scale_r", "feasible");

    for (int k = 0; k <= steps; ++k) {
      const double t = k * bike.dt();
      if (sc.friction_update) sc.friction_update(bike, t);
      const auto u = sc.control(t, bike.state());
      bike.step(u);
      const auto& x = bike.state();
      w.writeRow(sc.name, k, t, x[0], x[1], x[2], x[3], x[4], x[5], u[0], u[1], bike.muScaleFront(), bike.muScaleRear(), bike.isFeasible() ? 1 : 0);
      if (!bike.isFeasible()) break;
    }

    std::cout << "Validation wrote: " << csv_path << "\n";
  }

  if (!ran_any) {
    std::cerr << "No validation scenarios selected. validate_scenarios=" << list << "\n";
    return false;
  }

  // Best-effort plotting
  const fs::path script = fs::path("cpp") / "validation" / "plot_open_loop.py";
  if (fs::exists(script)) {
    const fs::path out_png = vdir / "open_loop_summary.png";
    const std::string cmd = "python3 " + shellQuote(script.string()) + " --dir " + shellQuote(vdir.string()) + " --out " + shellQuote(out_png.string());
    (void)std::system(cmd.c_str());
  }

  return true;
}

bool runNmpcValidation(BikeModel& bike, const KeyValueConfig& cfg, const fs::path& out_dir) {
  const fs::path vdir = out_dir / "validation" / "nmpc";
  fs::create_directories(vdir);

  // Initial state
  const Vec<BikeModel::NX> x0{cfg.getDouble("start_x", 0.0),
                              cfg.getDouble("start_y", 0.0),
                              cfg.getDouble("start_psi", 0.0),
                              cfg.getDouble("start_vx", 3.0),
                              cfg.getDouble("start_vy", 0.0),
                              cfg.getDouble("start_r", 0.0)};

  // Very wide bounds (open space)
  Vec<BikeModel::NX> xmin{}, xmax{};
  xmin.fill(-1e9);
  xmax.fill(+1e9);
  bike.setBounds(xmin, xmax);

  const double delta_max = cfg.getDouble("delta_max", deg2rad(30.0));
  const double sfx_max = cfg.getDouble("sfx_max", 0.1);

  const double T = cfg.getDouble("nmpc_val_T", 10.0);
  const double sfx_ref = std::clamp(cfg.getDouble("nmpc_val_sfx_ref", 0.01), -sfx_max, sfx_max);
  const double delta_const = std::clamp(cfg.getDouble("nmpc_val_delta_const", deg2rad(10.0)), -delta_max, delta_max);
  const double delta_amp = std::clamp(cfg.getDouble("nmpc_val_delta_sine_amp", deg2rad(8.0)), -delta_max, delta_max);
  const double delta_hz = cfg.getDouble("nmpc_val_delta_sine_hz", 0.25);

  // Build scenarios (reference is generated by simulating the model under u_ref)
  std::vector<NmpcValScenario> all;
  all.push_back(NmpcValScenario{
      "ref_straight",
      T,
      [=](double /*t*/, const Vec<BikeModel::NX>& /*x*/) { return Vec<BikeModel::NU>{0.0, sfx_ref}; }});
  all.push_back(NmpcValScenario{
      "ref_constant_steer",
      T,
      [=](double /*t*/, const Vec<BikeModel::NX>& /*x*/) { return Vec<BikeModel::NU>{delta_const, sfx_ref}; }});
  all.push_back(NmpcValScenario{
      "ref_sine_steer",
      T,
      [=](double t, const Vec<BikeModel::NX>& /*x*/) {
        const double d = delta_amp * std::sin(2.0 * M_PI * delta_hz * t);
        return Vec<BikeModel::NU>{std::clamp(d, -delta_max, delta_max), sfx_ref};
      }});

  const auto wanted = splitCsvList(cfg.getString("nmpc_val_scenarios", "ref_straight,ref_constant_steer,ref_sine_steer"));
  auto wantIt = [&](const std::string& name) {
    if (wanted.empty()) return true;
    return std::find(wanted.begin(), wanted.end(), name) != wanted.end();
  };

  // NMPC params
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
  np.delta_max = delta_max;
  np.sfx_max = sfx_max;

  bool ran_any = false;
  CsvWriter summary(vdir / "nmpc_validation_summary.csv");
  summary.writeRow("scenario", "T", "rms_pos_err", "final_pos_err", "final_vx_err");

  for (const auto& sc : all) {
    if (!wantIt(sc.name)) continue;
    ran_any = true;

    // Reference generation
    BikeModel ref_model = bike;
    ref_model.reset(x0, Vec<BikeModel::NU>{0.0, 0.0});
    std::vector<Vec<BikeModel::NX>> ref;
    const int steps = std::max(1, static_cast<int>(std::floor(sc.duration_s / ref_model.dt() + 1e-9)));
    ref.reserve(static_cast<std::size_t>(steps + 1));
    ref.push_back(ref_model.state());
    std::vector<Vec<BikeModel::NU>> uref;
    uref.reserve(static_cast<std::size_t>(steps + 1));
    uref.push_back(Vec<BikeModel::NU>{0.0, 0.0});

    for (int k = 1; k <= steps; ++k) {
      const double t = k * ref_model.dt();
      const auto u = sc.u_ref(t, ref_model.state());
      ref_model.step(u);
      ref.push_back(ref_model.state());
      uref.push_back(u);
      if (!ref_model.isFeasible()) break;
    }

    // Track with NMPC
    NmpcTracker tracker(bike, np);
    const auto exec = tracker.track(ref);

    // Write CSVs
    {
      CsvWriter w(vdir / (sc.name + "_ref.csv"));
      w.writeRow("k", "t", "x", "y", "psi", "vx", "vy", "r", "u_ref_delta", "u_ref_sfx");
      for (std::size_t k = 0; k < ref.size(); ++k) {
        const auto& x = ref[k];
        const auto& u = uref[k];
        w.writeRow(static_cast<int>(k), k * ref_model.dt(), x[0], x[1], x[2], x[3], x[4], x[5], u[0], u[1]);
      }
    }
    {
      CsvWriter w(vdir / (sc.name + "_exec.csv"));
      w.writeRow("k", "t", "x", "y", "psi", "vx", "vy", "r", "u_delta", "u_sfx", "x_ref", "y_ref", "psi_ref", "vx_ref");
      for (std::size_t k = 0; k < exec.size(); ++k) {
        const auto& s = exec[k];
        w.writeRow(static_cast<int>(k), s.t, s.x[0], s.x[1], s.x[2], s.x[3], s.x[4], s.x[5], s.u[0], s.u[1], s.x_ref[0], s.x_ref[1], s.x_ref[2],
                   s.x_ref[3]);
      }
    }

    // Metrics
    double sum_e2 = 0.0;
    double final_e = 0.0;
    double final_vx_e = 0.0;
    const std::size_t N = exec.size();
    for (std::size_t k = 0; k < N; ++k) {
      const auto& s = exec[k];
      const double ex = s.x[0] - s.x_ref[0];
      const double ey = s.x[1] - s.x_ref[1];
      const double e = std::sqrt(ex * ex + ey * ey);
      sum_e2 += e * e;
      if (k + 1 == N) {
        final_e = e;
        final_vx_e = s.x[3] - s.x_ref[3];
      }
    }
    const double rms = (N > 0) ? std::sqrt(sum_e2 / static_cast<double>(N)) : 1e9;
    summary.writeRow(sc.name, sc.duration_s, rms, final_e, final_vx_e);
  }

  if (!ran_any) {
    std::cerr << "No NMPC validation scenarios selected. nmpc_val_scenarios=" << cfg.getString("nmpc_val_scenarios", "") << "\n";
    return false;
  }

  // Plot
  const fs::path script = fs::path("cpp") / "validation" / "plot_nmpc_validation.py";
  if (fs::exists(script)) {
    const fs::path out_png = vdir / "nmpc_validation.png";
    const std::string cmd = "python3 " + shellQuote(script.string()) + " --dir " + shellQuote(vdir.string()) + " --out " + shellQuote(out_png.string());
    (void)std::system(cmd.c_str());
  }
  return true;
}

bool runOpenLoopValidationFourWheel(FourWheelModel& model, const KeyValueConfig& cfg, const fs::path& out_dir) {
  const fs::path vdir = out_dir / "validation" / "four_wheel";
  fs::create_directories(vdir);

  // Base initial state and input
  // x = [x y psi vx vy r dphi_fl dphi_fr dphi_rl dphi_rr]
  const Vec<FourWheelModel::NX> x0{cfg.getDouble("start_x", 0.0),
                                   cfg.getDouble("start_y", 0.0),
                                   cfg.getDouble("start_psi", 0.0),
                                   cfg.getDouble("start_vx", 10.0),
                                   cfg.getDouble("start_vy", 0.0),
                                   cfg.getDouble("start_r", 0.0),
                                   cfg.getDouble("start_dphi_fl", 0.0),
                                   cfg.getDouble("start_dphi_fr", 0.0),
                                   cfg.getDouble("start_dphi_rl", 0.0),
                                   cfg.getDouble("start_dphi_rr", 0.0)};
  const Vec<FourWheelModel::NU> u0{0.0, 0.0, 0.0, 0.0, 0.0};

  // Open-space bounds (wide)
  Vec<FourWheelModel::NX> xmin{}, xmax{};
  xmin.fill(cfg.getDouble("validate4_xmin", -1e6));
  xmax.fill(cfg.getDouble("validate4_xmax", +1e6));
  model.setBounds(xmin, xmax);

  const double delta_max = cfg.getDouble("delta_max", deg2rad(30.0));

  // Map existing "sfx-like" longitudinal command into wheel torques
  // (configurable drive layout: RWD/FWD/AWD).
  const double tau_max = cfg.getDouble("validate4_tau_max", 1500.0);
  const double accel_tau = cfg.getDouble("validate4_accel_tau", 0.6 * tau_max);
  const double brake_tau = cfg.getDouble("validate4_brake_tau", -0.6 * tau_max);
  const double cruise_tau = cfg.getDouble("validate4_cruise_tau", 0.15 * tau_max);

  auto upper = [](std::string s) {
    for (auto& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
  };
  const std::string drive = upper(cfg.getString("validate4_drive", "RWD"));
  const double split_front = std::clamp(cfg.getDouble("validate4_split_front", 0.5), 0.0, 1.0); // AWD only

  auto distributeTorque = [&](double tau_total) {
    // tau_total is "total" axle torque magnitude for the driven axle(s).
    // We apply equal left/right on each driven axle.
    const double tt = std::clamp(tau_total, -tau_max, tau_max);
    double tfl = 0.0, tfr = 0.0, trl = 0.0, trr = 0.0;
    if (drive == "FWD") {
      tfl = 0.5 * tt;
      tfr = 0.5 * tt;
    } else if (drive == "AWD") {
      const double tf = split_front * tt;
      const double tr = (1.0 - split_front) * tt;
      tfl = 0.5 * tf;
      tfr = 0.5 * tf;
      trl = 0.5 * tr;
      trr = 0.5 * tr;
    } else { // RWD default
      trl = 0.5 * tt;
      trr = 0.5 * tt;
    }
    return Vec<FourWheelModel::NU>{0.0, tfl, tfr, trl, trr};
  };

  const double steer_const = cfg.getDouble("validate_steer_const_rad", deg2rad(10.0));
  const double steer_amp = cfg.getDouble("validate_steer_sine_amp_rad", deg2rad(8.0));
  const double steer_freq = cfg.getDouble("validate_steer_sine_hz", 0.25);

  const double t_accel = cfg.getDouble("validate_t_accel", 6.0);
  const double t_brake = cfg.getDouble("validate_t_brake", 4.0);
  const double t_turn = cfg.getDouble("validate_t_turn", 8.0);
  const double t_sine = cfg.getDouble("validate_t_sine", 10.0);
  const double t_mu_drop = cfg.getDouble("validate_t_mu_drop", 10.0);

  // Optional: cut longitudinal torque to zero after some time for selected scenarios.
  const double lon_cutoff_s = cfg.getDouble("validate_lon_cutoff_s", -1.0);
  const auto lon_cutoff_scenarios = splitCsvList(cfg.getString("validate_lon_cutoff_scenarios", "turn_sine_steer,mu_drop_sine_steer"));
  auto useLonCutoff = [&](const std::string& sc_name, double t) {
    if (lon_cutoff_s < 0.0) return false;
    if (std::find(lon_cutoff_scenarios.begin(), lon_cutoff_scenarios.end(), sc_name) == lon_cutoff_scenarios.end()) return false;
    return t >= lon_cutoff_s;
  };

  const double mu_f_start = cfg.getDouble("validate_mu_f_start", model.muScaleFront());
  const double mu_r_start = cfg.getDouble("validate_mu_r_start", model.muScaleRear());
  const double mu_f_end = cfg.getDouble("validate_mu_f_end", 0.6);
  const double mu_r_end = cfg.getDouble("validate_mu_r_end", 0.6);

  // Define scenarios
  struct Sc4 {
    std::string name;
    double duration_s;
    std::function<Vec<FourWheelModel::NU>(double t, const Vec<FourWheelModel::NX>& x)> control;
    std::function<void(FourWheelModel&, double)> friction_update;
  };
  std::vector<Sc4> all;

  // Motor torque drop with speed (simple actuator model)
  const std::string torque_model = upper(cfg.getString("validate4_torque_speed_model", "linear")); // CONSTANT|LINEAR|HYPERBOLIC
  const double torque_vmax = cfg.getDouble("validate4_torque_vmax_mps", 40.0);
  const double torque_vref = cfg.getDouble("validate4_torque_vref_mps", 20.0); // for hyperbolic
  const double torque_min_factor = std::clamp(cfg.getDouble("validate4_torque_min_factor", 0.0), 0.0, 1.0);
  const bool apply_drop_to_brake = cfg.getBool("validate4_apply_drop_to_brake", false);

  auto motorTorqueAvailable = [&](double tau_cmd, double vx_now) {
    // Apply drop primarily for positive (drive) torque. Regen braking may also drop, but friction brakes do not.
    if (!(tau_cmd > 0.0 || apply_drop_to_brake)) return std::clamp(tau_cmd, -tau_max, tau_max);
    const double v = std::abs(vx_now);
    double f = 1.0;
    if (torque_model == "CONSTANT") {
      f = 1.0;
    } else if (torque_model == "HYPERBOLIC") {
      const double vr = std::max(1e-6, torque_vref);
      f = 1.0 / (1.0 + v / vr);
    } else { // LINEAR default
      const double vm = std::max(1e-6, torque_vmax);
      f = 1.0 - (v / vm);
    }
    f = std::clamp(f, torque_min_factor, 1.0);
    return std::clamp(tau_cmd * f, -tau_max, tau_max);
  };
  all.push_back(Sc4{
      "accel_straight",
      t_accel,
      [=](double /*t*/, const Vec<FourWheelModel::NX>& x) {
        auto u = distributeTorque(motorTorqueAvailable(accel_tau, x[3]));
        u[0] = 0.0;
        return u;
      },
      {}});
  all.push_back(Sc4{
      "brake_straight",
      t_brake,
      [=](double /*t*/, const Vec<FourWheelModel::NX>& x) {
        auto u = distributeTorque(motorTorqueAvailable(brake_tau, x[3]));
        u[0] = 0.0;
        return u;
      },
      {}});
  all.push_back(Sc4{
      "turn_constant_steer",
      t_turn,
      [=](double /*t*/, const Vec<FourWheelModel::NX>& x) {
        const double d = std::clamp(steer_const, -delta_max, delta_max);
        auto u = distributeTorque(motorTorqueAvailable(cruise_tau, x[3]));
        u[0] = d;
        return u;
      },
      {}});
  all.push_back(Sc4{
      "turn_sine_steer",
      t_sine,
      [=](double t, const Vec<FourWheelModel::NX>& x) {
        const double d = std::clamp(steer_amp * std::sin(2.0 * M_PI * steer_freq * t), -delta_max, delta_max);
        const double tau_cmd = useLonCutoff("turn_sine_steer", t) ? 0.0 : cruise_tau;
        auto u = distributeTorque(motorTorqueAvailable(tau_cmd, x[3]));
        u[0] = d;
        return u;
      },
      {}});
  all.push_back(Sc4{
      "mu_drop_sine_steer",
      t_mu_drop,
      [=](double t, const Vec<FourWheelModel::NX>& x) {
        const double d = std::clamp(steer_amp * std::sin(2.0 * M_PI * steer_freq * t), -delta_max, delta_max);
        const double tau_cmd = useLonCutoff("mu_drop_sine_steer", t) ? 0.0 : cruise_tau;
        auto u = distributeTorque(motorTorqueAvailable(tau_cmd, x[3]));
        u[0] = d;
        return u;
      },
      [=](FourWheelModel& m, double t) {
        const double a = (t_mu_drop <= 0.0) ? 1.0 : std::clamp(t / t_mu_drop, 0.0, 1.0);
        const double muf = (1.0 - a) * mu_f_start + a * mu_f_end;
        const double mur = (1.0 - a) * mu_r_start + a * mu_r_end;
        m.setFrictionScales(muf, mur);
      }});

  // Choose scenarios
  const std::string list = cfg.getString("validate_scenarios", "accel_straight,brake_straight,turn_constant_steer,turn_sine_steer,mu_drop_sine_steer");
  const auto wanted = splitCsvList(list);
  auto wantIt = [&](const std::string& name) {
    if (wanted.empty()) return true;
    return std::find(wanted.begin(), wanted.end(), name) != wanted.end();
  };

  bool ran_any = false;
  for (const auto& sc : all) {
    if (!wantIt(sc.name)) continue;
    ran_any = true;

    model.reset(x0, u0);
    model.setFrictionScales(mu_f_start, mu_r_start);

    const int steps = std::max(1, static_cast<int>(std::floor(sc.duration_s / model.dt() + 1e-9)));
    const fs::path csv_path = vdir / (sc.name + ".csv");
    CsvWriter w(csv_path);
    w.writeRow("scenario", "k", "t", "x", "y", "psi", "vx", "vy", "r", "u_delta", "tau_fl", "tau_fr", "tau_rl", "tau_rr", "mu_scale_f", "mu_scale_r",
               "feasible");

    for (int k = 0; k <= steps; ++k) {
      const double t = k * model.dt();
      if (sc.friction_update) sc.friction_update(model, t);
      const auto u = sc.control(t, model.state());
      model.step(u);
      const auto& x = model.state();
      w.writeRow(sc.name, k, t, x[0], x[1], x[2], x[3], x[4], x[5], u[0], u[1], u[2], u[3], u[4], model.muScaleFront(), model.muScaleRear(),
                 model.isFeasible() ? 1 : 0);
      if (!model.isFeasible()) break;
    }

    std::cout << "Validation wrote: " << csv_path << "\n";
  }

  if (!ran_any) {
    std::cerr << "No validation scenarios selected. validate_scenarios=" << list << "\n";
    return false;
  }

  const fs::path script = fs::path("cpp") / "validation" / "plot_open_loop.py";
  if (fs::exists(script)) {
    const fs::path out_png = vdir / "open_loop_summary.png";
    const std::string cmd = "python3 " + shellQuote(script.string()) + " --dir " + shellQuote(vdir.string()) + " --out " + shellQuote(out_png.string());
    (void)std::system(cmd.c_str());
  }

  return true;
}

} // namespace ru_racer::app

