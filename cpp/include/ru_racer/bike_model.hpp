#pragma once

#include "ru_racer/math.hpp"

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
  Vec<NX> x_ = vzeros<NX>();
  Vec<NU> u_ = vzeros<NU>();
  Vec<NX> goal_ = vzeros<NX>();

  Vec<NX> bounds_x_min_{};
  Vec<NX> bounds_x_max_{};
  Vec<NX> bounds_dx_min_{};
  Vec<NX> bounds_dx_max_{};

  std::vector<CircleObstacle> circles_{};
  const OccupancyGrid* grid_ = nullptr;

  bool feasible_ = true;
  double last_cost_ = 0.0;
};

} // namespace ru_racer

