#pragma once

#include "ru_racer/bike_model.hpp" // for FrictionConfig
#include "ru_racer/math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace ru_racer {

// Reuse the same online-updatable friction config concept (scale on tire peak).
// This matches the approach used for BikeModel, and is applied to the 4-wheel Pacejka D parameter.
// (Defined in bike_model.hpp)
// FrictionConfig is defined in bike_model.hpp

inline double clampAbs(double x, double m) { return std::copysign(std::min(std::abs(x), m), x); }

struct FourWheelParams {
  double dt = 0.02;
  double g = 9.81;

  // Vehicle constants (body)
  double m = 1500.0;
  double Izz = 2500.0;
  double lf = 1.2;
  double lr = 1.6;
  double b = 1.6;   // track width
  double h = 0.5;   // CG height

  // Wheel/tire
  double tire_r = 0.30;
  double wheel_J = 1.5; // wheel inertia

  // Pacejka-like params used in matlab/wheel_model.m (fricx/fricy)
  double tire_B = 11.275;
  double tire_C = 1.56;
  double tire_D = -0.95;
  double tire_E = -1.999;

  // Limits / safety
  double accel_limit = 20.0; // clamp dvx,dvy,dr magnitude (matches MATLAB dotz(4:6) clamp at 20)
};

class FourWheelModel {
 public:
  static constexpr std::size_t NX = 10;
  static constexpr std::size_t NU = 5; // [delta, tau_fl, tau_fr, tau_rl, tau_rr]

  explicit FourWheelModel(FourWheelParams p = {}) : p_(p) {
    bounds_x_min_.fill(-1e9);
    bounds_x_max_.fill(+1e9);
  }

  void setBounds(const Vec<NX>& xmin, const Vec<NX>& xmax) {
    bounds_x_min_ = xmin;
    bounds_x_max_ = xmax;
  }

  void setFrictionConfig(const FrictionConfig& fc);
  void setFrictionScales(double mu_scale_f, double mu_scale_r);
  double muScaleFront() const { return mu_scale_f_; }
  double muScaleRear() const { return mu_scale_r_; }

  void setClampVxNonnegative(bool on) { clamp_vx_nonnegative_ = on; }
  bool clampVxNonnegative() const { return clamp_vx_nonnegative_; }

  const Vec<NX>& state() const { return x_; }
  const Vec<NU>& input() const { return u_; }
  bool isFeasible() const { return feasible_; }
  double dt() const { return p_.dt; }

  void reset(const Vec<NX>& x0, const Vec<NU>& u0);

  Vec<NX> dynamics(const Vec<NX>& x, const Vec<NU>& u) const;
  Vec<NX> step(const Vec<NU>& u);

 private:
  bool checkFeasible(const Vec<NX>& x) const;

  // Helpers mirroring matlab/wheel_model.m
  Vec<4> wheelVelocitiesBody(const Vec<NX>& x, double delta) const;
  Vec<4> alpha(const Vec<NX>& x, double delta) const;
  Vec<4> lambda(const Vec<NX>& x, const Vec<4>& wv) const;
  Vec<4> normalLoads(const Vec<NX>& x, const Vec<NX>& last_dotz) const;
  Vec<4> fricx(const Vec<4>& lambda, const Vec<4>& fz, bool is_front) const;
  Vec<4> fricy(const Vec<4>& alpha, const Vec<4>& fz) const;

  FourWheelParams p_{};
  FrictionConfig friction_cfg_{};
  double mu_scale_f_ = 1.0;
  double mu_scale_r_ = 1.0;
  bool clamp_vx_nonnegative_ = false;

  Vec<NX> x_ = vzeros<NX>();
  Vec<NU> u_ = vzeros<NU>();
  Vec<NX> last_dotz_ = vzeros<NX>(); // used for load transfer (matches dotzo)

  Vec<NX> bounds_x_min_{};
  Vec<NX> bounds_x_max_{};

  bool feasible_ = true;
};

} // namespace ru_racer

