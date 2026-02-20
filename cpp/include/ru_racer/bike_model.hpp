#pragma once

#include "ru_racer/math.hpp"

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

namespace ru_racer {

struct CircleObstacle {
  double x{};
  double y{};
  double r{};
};

struct OccupancyGrid;

struct BikeParams {
  double dt = 0.02;
  double rewiring_value = 0.05;

  double m = 6.8;
  double Izz = 0.15;
  double lf = 0.15;
  double lr = 0.25;
  double g = 9.8;
  double h = 0.02;

  // Magic tire parameters as used in `Bike.m`
  struct Tire { double B{}, C{}, D{}; };
  Tire tirer{10.275, 1.56, -0.79};
  Tire tiref{10.275, 1.56, -0.85};
};

// Friction parameters that can be updated online during a run (e.g., from an estimator).
// We model this as a scale factor on the Magic Formula peak (D) front/rear.
struct FrictionConfig {
  double mu_scale_f0 = 1.0;
  double mu_scale_r0 = 1.0;
  double mu_scale_f_min = 0.3;
  double mu_scale_f_max = 1.5;
  double mu_scale_r_min = 0.3;
  double mu_scale_r_max = 1.5;

  // If enabled, friction authority fades near zero speed to avoid unphysical sign flips when braking.
  // Effective scale = mu_scale_* * (1 - exp(-|vx| / mu_speed_vscale_mps))
  bool mu_speed_dependent = false;
  double mu_speed_vscale_mps = 1.0;
};

class BikeModel {
 public:
  static constexpr std::size_t NX = 6;
  static constexpr std::size_t NU = 2;

  explicit BikeModel(BikeParams p = {}) : p_(p) {
    bounds_x_min_.fill(-25.0);
    bounds_x_max_.fill(+25.0);
    bounds_dx_min_.fill(-20.0);
    bounds_dx_max_.fill(+20.0);
  }

  void setGoal(const Vec<NX>& goal) { goal_ = goal; }
  void setBounds(const Vec<NX>& xmin, const Vec<NX>& xmax) {
    bounds_x_min_ = xmin;
    bounds_x_max_ = xmax;
  }
  void setCircleObstacles(std::vector<CircleObstacle> obs) { circles_ = std::move(obs); }
  void setOccupancyGrid(const OccupancyGrid* grid) { grid_ = grid; }

  // Optional vx limits used only when an occupancy grid is active (mirrors MATLAB map feasibility checks).
  // Defaults match the original small-car track settings.
  void setMapVxLimits(double vx_min, double vx_max) {
    map_vx_min_ = vx_min;
    map_vx_max_ = vx_max;
  }
  double mapVxMin() const { return map_vx_min_; }
  double mapVxMax() const { return map_vx_max_; }

  // Runtime friction update hooks (do not affect static vehicle params like mass/geometry).
  void setFrictionConfig(const FrictionConfig& fc) {
    friction_cfg_ = fc;
    mu_scale_f_ = fc.mu_scale_f0;
    mu_scale_r_ = fc.mu_scale_r0;
  }
  void setFrictionScales(double mu_scale_f, double mu_scale_r) {
    mu_scale_f_ = std::clamp(mu_scale_f, friction_cfg_.mu_scale_f_min, friction_cfg_.mu_scale_f_max);
    mu_scale_r_ = std::clamp(mu_scale_r, friction_cfg_.mu_scale_r_min, friction_cfg_.mu_scale_r_max);
  }
  double muScaleFront() const { return mu_scale_f_; }
  double muScaleRear() const { return mu_scale_r_; }

  void setClampVxNonnegative(bool on) { clamp_vx_nonnegative_ = on; }
  bool clampVxNonnegative() const { return clamp_vx_nonnegative_; }

  const Vec<NX>& state() const { return x_; }
  const Vec<NU>& input() const { return u_; }
  bool isFeasible() const { return feasible_; }
  double lastCost() const { return last_cost_; }
  double dt() const { return p_.dt; }

  void reset(const Vec<NX>& x0, const Vec<NU>& u0);

  Vec<NX> dynamics(const Vec<NX>& x, const Vec<NU>& u) const;

  // Single integration step (dt). Does NOT apply early-stop-on-goal; does update feasibility.
  Vec<NX> step(const Vec<NU>& u);

  // Integrate forward with constant u for [0, horizon] (inclusive steps like MATLAB loop).
  // Returns the final state.
  Vec<NX> simulateConstantInput(const Vec<NU>& u, double horizon);

  // Closed-loop "Controller" from `Bike.m`, using state feedback to track Xd.
  Vec<NU> controllerTo(const Vec<NX>& xd) const;

  // Integrate forward with controller tracking xd (recomputed each dt).
  Vec<NX> simulateClosedLoopTo(const Vec<NX>& xd, double horizon);

 private:
  bool checkFeasible(const Vec<NX>& x, const Vec<NX>& dx) const;

  BikeParams p_{};
  FrictionConfig friction_cfg_{};
  double mu_scale_f_ = 1.0;
  double mu_scale_r_ = 1.0;
  bool clamp_vx_nonnegative_ = false;
  Vec<NX> x_ = vzeros<NX>();
  Vec<NU> u_ = vzeros<NU>();
  Vec<NX> goal_ = vzeros<NX>();

  Vec<NX> bounds_x_min_{};
  Vec<NX> bounds_x_max_{};
  Vec<NX> bounds_dx_min_{};
  Vec<NX> bounds_dx_max_{};

  std::vector<CircleObstacle> circles_{};
  const OccupancyGrid* grid_ = nullptr;

  double map_vx_min_ = 0.0;
  double map_vx_max_ = 5.0;

  bool feasible_ = true;
  double last_cost_ = 0.0;
};

} // namespace ru_racer

