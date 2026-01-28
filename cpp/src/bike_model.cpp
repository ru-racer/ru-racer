#include "ru_racer/bike_model.hpp"

#include "ru_racer/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>

namespace ru_racer {

void BikeModel::reset(const Vec<NX>& x0, const Vec<NU>& u0) {
  x_ = x0;
  u_ = u0;
  feasible_ = true;
  last_cost_ = 0.0;
}

Vec<BikeModel::NX> BikeModel::step(const Vec<NU>& u) {
  feasible_ = true;
  u_ = u;
  const auto dx = dynamics(x_, u_);
  x_ = add(x_, scale(dx, p_.dt));
  feasible_ = checkFeasible(x_, dx);
  return x_;
}

Vec<BikeModel::NX> BikeModel::dynamics(const Vec<NX>& x, const Vec<NU>& u) const {
  // x = [x y psi vx vy dotpsi]
  const double psi = x[2];
  const double vx = x[3];
  const double vy = x[4];
  const double dotpsi = x[5];

  const double delta = u[0];
  const double sfx = u[1];
  const double srx = sfx;

  const double cd = std::cos(delta);
  const double sd = std::sin(delta);

  // Match MATLAB equations.
  const double denom_f = (vx * cd + (vy + p_.lf * dotpsi) * sd);
  const double sfy = denom_f == 0.0 ? 0.0 : (((vy + p_.lf * dotpsi) * cd - vx * sd) / denom_f);
  const double sry = (vx == 0.0) ? 0.0 : (vy - p_.lr * dotpsi) / vx;

  const double sf = std::sqrt(sfy * sfy + sfx * sfx);
  const double sr = std::sqrt(sry * sry + srx * srx);

  const auto mur = p_.tirer.D * std::sin(p_.tirer.C * std::atan(p_.tirer.B * sr));
  const auto muf = p_.tiref.D * std::sin(p_.tiref.C * std::atan(p_.tiref.B * sf));

  double mufx = 0.0, mufy = 0.0;
  if (std::abs(sf) > 0.01) {
    mufx = -muf * (sfx / sf);
    mufy = -muf * (sfy / sf);
  }

  double murx = 0.0, mury = 0.0;
  if (std::abs(sr) > 0.001) {
    murx = -mur * (srx / sr);
    mury = -mur * (sry / sr);
  }

  const double denom_fz = (p_.lf + p_.lr + p_.h * (mufx * cd - mufy * sd - murx));
  const double Ffz = denom_fz == 0.0
                         ? (p_.m * p_.g * (p_.lr - murx * p_.h))
                         : (p_.m * p_.g * (p_.lr - murx * p_.h)) / denom_fz;
  const double Frz = p_.m * p_.g - Ffz;

  const double Ffx = mufx * Ffz;
  const double Frx = murx * Frz;
  const double Ffy = -mufy * Ffz;
  const double Fry = -mury * Frz;

  const double dotx = vx * std::cos(psi) - vy * std::sin(psi);
  const double doty = vx * std::sin(psi) + vy * std::cos(psi);
  const double dotvx = (Ffx * cd - Ffy * sd + Frx) / p_.m + vy * dotpsi;
  const double dotvy = (Ffx * sd + Ffy * cd + Fry) / p_.m - vx * dotpsi;
  const double dotdotpsi = (p_.lf * (Ffx * sd + Ffy * cd) - p_.lr * Fry) / p_.Izz;

  return Vec<NX>{dotx, doty, dotpsi, dotvx, dotvy, dotdotpsi};
}

bool BikeModel::checkFeasible(const Vec<NX>& x, const Vec<NX>& dx) const {
  // Bounds on state.
  for (std::size_t i = 0; i < NX; ++i) {
    if (!(x[i] > bounds_x_min_[i] && x[i] < bounds_x_max_[i])) return false;
  }

  // Optional bounds on derivatives (disabled in MATLAB Bike.check_feasibility).
  (void)dx;

  // Circle obstacles.
  for (const auto& o : circles_) {
    const double d2 = (x[0] - o.x) * (x[0] - o.x) + (x[1] - o.y) * (x[1] - o.y);
    const double rr = (o.r * 1.1) * (o.r * 1.1);
    if (d2 < rr) return false;
  }

  // Occupancy grid feasibility (if provided): also mimic Bike.check_feasibility_map vx limits.
  if (grid_) {
    const double vx = x[3];
    if (vx <= 0.0) return false;
    if (vx >= 5.0) return false;
    if (!grid_->isFreeWorld(x[0], x[1])) return false;
  }

  return true;
}

Vec<BikeModel::NX> BikeModel::simulateConstantInput(const Vec<NU>& u, double horizon) {
  feasible_ = true;
  u_ = u;
  last_cost_ = 0.0;

  // MATLAB uses: for t=0:dt:Horizon
  const int steps = std::max(0, static_cast<int>(std::floor(horizon / p_.dt + 1e-9)));
  for (int k = 0; k <= steps; ++k) {
    const double t = k * p_.dt;
    step(u_);

    if (norm2(sub(Vec<2>{x_[0], x_[1]}, Vec<2>{goal_[0], goal_[1]})) < 2.0 * p_.rewiring_value) {
      last_cost_ = t;
      return x_;
    }
    if (!feasible_) {
      last_cost_ = 1000.0;
      return x_;
    }
  }
  last_cost_ = horizon;
  return x_;
}

Vec<BikeModel::NU> BikeModel::controllerTo(const Vec<NX>& xd) const {
  // From Bike.m Controller:
  // ke=.3; kp1=2; kp2=.01;
  const double ke = 0.3;
  const double kp1 = 2.0;
  const double kp2 = 0.01;

  const auto& x = x_;
  const double derr = std::atan2(xd[1] - x[1], xd[0] - x[0]);
  const double kp = kp1 * std::exp(-kp2 * std::abs(x[3]));

  const double delta = sat((kp * (-x[2] + derr)) + ke * std::sin(xd[2] - x[2]), M_PI / 9.0);

  Vec<NU> u{};
  u[0] = delta;
  u[1] = sat(0.1 * std::exp(std::abs(derr)) *
                 (0.1 * (xd[3] - x[3]) + std::copysign(1.0, derr + 55.0) * 0.07 * norm2(sub(Vec<2>{xd[0], xd[1]}, Vec<2>{x[0], x[1]}))),
             0.1);
  return u;
}

Vec<BikeModel::NX> BikeModel::simulateClosedLoopTo(const Vec<NX>& xd, double horizon) {
  feasible_ = true;
  // MATLAB uses: for t=0:dt:Horizon (and sets Last_Cost += dt); we'll track time similarly.
  const int steps = std::max(0, static_cast<int>(std::floor(horizon / p_.dt + 1e-9)));
  last_cost_ = 0.0;

  for (int k = 0; k <= steps; ++k) {
    u_ = controllerTo(xd);
    step(u_);
    last_cost_ += p_.dt;

    if (norm2(sub(Vec<3>{xd[0], xd[1], xd[2]}, Vec<3>{x_[0], x_[1], x_[2]})) < p_.rewiring_value) {
      feasible_ = true;
      return x_;
    }

    if (!feasible_) {
      last_cost_ = 1000.0;
      return x_;
    }
  }
  last_cost_ = horizon;
  return x_;
}

} // namespace ru_racer

