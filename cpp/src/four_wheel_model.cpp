#include "ru_racer/four_wheel_model.hpp"

#include <algorithm>
#include <cmath>

namespace ru_racer {

void FourWheelModel::setFrictionConfig(const FrictionConfig& fc) {
  friction_cfg_ = fc;
  mu_scale_f_ = fc.mu_scale_f0;
  mu_scale_r_ = fc.mu_scale_r0;
}

void FourWheelModel::setFrictionScales(double mu_scale_f, double mu_scale_r) {
  mu_scale_f_ = std::clamp(mu_scale_f, friction_cfg_.mu_scale_f_min, friction_cfg_.mu_scale_f_max);
  mu_scale_r_ = std::clamp(mu_scale_r, friction_cfg_.mu_scale_r_min, friction_cfg_.mu_scale_r_max);
}

void FourWheelModel::reset(const Vec<NX>& x0, const Vec<NU>& u0) {
  x_ = x0;
  u_ = u0;
  last_dotz_ = vzeros<NX>();
  feasible_ = true;
}

bool FourWheelModel::checkFeasible(const Vec<NX>& x) const {
  for (std::size_t i = 0; i < NX; ++i) {
    if (!(x[i] > bounds_x_min_[i] && x[i] < bounds_x_max_[i])) return false;
  }
  return true;
}

Vec<4> FourWheelModel::wheelVelocitiesBody(const Vec<NX>& x, double delta) const {
  // Matches Wheel_velocity() in matlab/wheel_model.m
  const double vx = x[3];
  const double vy = x[4];
  const double r = x[5];
  const double sd = std::sin(delta);
  const double cd = std::cos(delta);
  const double wf = sd * (vy + p_.lf * r) + vx * cd;
  return Vec<4>{wf, wf, vx, vx};
}

Vec<4> FourWheelModel::alpha(const Vec<NX>& x, double delta) const {
  // Matches Alpha() in matlab/wheel_model.m
  const double vx = x[3];
  const double vy = x[4];
  const double r = x[5];
  if (vx == 0.0) return Vec<4>{0.0, 0.0, 0.0, 0.0};
  const double alf = std::atan((vy + p_.lf * r) / vx) - delta;
  const double alr = std::atan((vy - p_.lr * r) / vx);
  return Vec<4>{alf, alf, alr, alr};
}

Vec<4> FourWheelModel::lambda(const Vec<NX>& x, const Vec<4>& wv) const {
  // Matches Lambda() in matlab/wheel_model.m
  const auto dphi = Vec<4>{x[6], x[7], x[8], x[9]};
  Vec<4> lam{};
  for (int i = 0; i < 4; ++i) {
    const double vwheel = p_.tire_r * dphi[static_cast<std::size_t>(i)];
    const double denom = std::max(vwheel, wv[static_cast<std::size_t>(i)]);
    lam[static_cast<std::size_t>(i)] = (denom == 0.0) ? 0.0 : (vwheel - wv[static_cast<std::size_t>(i)]) / denom;
  }
  return lam;
}

Vec<4> FourWheelModel::normalLoads(const Vec<NX>& /*x*/, const Vec<NX>& dotzo) const {
  // Matches F_z block in matlab/wheel_model.m (uses previous dotz = dotzo)
  const double L = p_.lf + p_.lr;
  const double dvx = dotzo[3];
  const double dvy = dotzo[4];

  const double m = p_.m;
  const double g = p_.g;
  const double h = p_.h;
  const double b = p_.b;

  const double common_f = g * p_.lr / L;
  const double common_r = g * p_.lf / L;

  const double aL = (dvx / L);
  const double aY = (dvy / b);

  return scale(Vec<4>{common_f - h * (aL + aY),
                      common_f - h * (aL - aY),
                      common_r - h * (aL - aY),
                      common_r - h * (aL + aY)},
               0.5 * m);
}

static double wrapAsinSin(double a) {
  // MATLAB uses asin(sin(alpha)) to wrap into [-pi/2, pi/2]
  return std::asin(std::sin(a));
}

Vec<4> FourWheelModel::fricx(const Vec<4>& lam, const Vec<4>& fz, bool is_front) const {
  // Port of fricx() from matlab/wheel_model.m
  Vec<4> fx{};
  for (int i = 0; i < 4; ++i) {
    const double lambda_i = lam[static_cast<std::size_t>(i)];
    const double alpha = -lambda_i;
    const double aB = wrapAsinSin(alpha);

    // Apply friction scaling on D, with optional speed-dependence like BikeModel.
    double vfac = 1.0;
    if (friction_cfg_.mu_speed_dependent) {
      // use current vx as proxy (wheel_model is in body frame)
      const double v = std::abs(x_[3]);
      const double vs = std::max(1e-6, friction_cfg_.mu_speed_vscale_mps);
      vfac = 1.0 - std::exp(-v / vs);
    }
    (void)is_front;
    const double mu = (i < 2 ? mu_scale_f_ : mu_scale_r_) * vfac;
    const double D = p_.tire_D * mu;

    const double f = D * std::sin(p_.tire_C * std::atan(p_.tire_B * (1.0 - p_.tire_E) * aB + p_.tire_E * std::atan(p_.tire_B * aB)));
    double fxi = f * fz[static_cast<std::size_t>(i)];

    if (fxi * alpha > 0.0) fxi = -fxi;
    // In MATLAB: for i=1:2 (front wheels) if fx>0 fx=0
    if (i < 2 && fxi > 0.0) fxi = 0.0;

    fx[static_cast<std::size_t>(i)] = fxi;
  }
  return fx;
}

Vec<4> FourWheelModel::fricy(const Vec<4>& al, const Vec<4>& fz) const {
  // Port of fricy() from matlab/wheel_model.m
  Vec<4> fy{};
  for (int i = 0; i < 4; ++i) {
    const double aB = wrapAsinSin(al[static_cast<std::size_t>(i)]);

    double vfac = 1.0;
    if (friction_cfg_.mu_speed_dependent) {
      const double v = std::abs(x_[3]);
      const double vs = std::max(1e-6, friction_cfg_.mu_speed_vscale_mps);
      vfac = 1.0 - std::exp(-v / vs);
    }
    const double mu = (i < 2 ? mu_scale_f_ : mu_scale_r_) * vfac;
    const double D = p_.tire_D * mu;

    const double f = D * std::sin(p_.tire_C * std::atan(p_.tire_B * (1.0 - p_.tire_E) * aB + p_.tire_E * std::atan(p_.tire_B * aB)));
    double fyi = f * fz[static_cast<std::size_t>(i)];
    if (fyi * al[static_cast<std::size_t>(i)] > 0.0) fyi = -fyi;
    fy[static_cast<std::size_t>(i)] = fyi;
  }
  return fy;
}

Vec<FourWheelModel::NX> FourWheelModel::dynamics(const Vec<NX>& x, const Vec<NU>& u) const {
  // Port of wheel_model() main equations.
  const double psi = x[2];
  const double vx = x[3];
  const double vy = x[4];
  const double r = x[5];
  const double delta = u[0];

  const double cp = std::cos(psi);
  const double sp = std::sin(psi);
  const double cd = std::cos(delta);
  const double sd = std::sin(delta);

  // Rotation J
  // dz1 = J * z2
  const double dx = cp * vx - sp * vy;
  const double dy = sp * vx + cp * vy;
  const double dpsi = r;

  // Coriolis-like term C (body-frame coupling)
  // Standard planar body dynamics:
  //   m*dvx = Fx + m*r*vy
  //   m*dvy = Fy - m*r*vx
  const double Cx = -vy * r * p_.m;
  const double Cy = +vx * r * p_.m;

  // Wheel velocities, slips
  const auto wv = wheelVelocitiesBody(x, delta);
  const auto al = alpha(x, delta);
  const auto lam = lambda(x, wv);
  auto fz = normalLoads(x, last_dotz_);
  // Physical clamp: normal loads cannot be negative.
  for (int i = 0; i < 4; ++i) fz[static_cast<std::size_t>(i)] = std::max(0.0, fz[static_cast<std::size_t>(i)]);

  // Forces per wheel
  const auto fx = fricx(lam, fz, true /*ignored per-wheel inside*/);
  const auto fy = fricy(al, fz);

  // B_x and B_y matrices (3x4) from MATLAB
  // We'll compute body-frame force result explicitly:
  // Fx_body = sum( c_d*Fx_front + 1*Fx_rear etc ) + ...
  // Following matlab:
  // B_x = [c_d c_d 1 1;
  //        s_d s_d 0 0;
  //        Lf*s_d-.5*b*c_d, Lf*s_d+.5*b*c_d, -b, b]
  // B_y = [-s_d -s_d 0 0;
  //         c_d  c_d 1 1;
  //         Lf*c_d-.5*b*s_d, Lf*c_d+.5*b*s_d, -Lr, -Lr]
  const double b = p_.b;
  const double Lf = p_.lf;
  const double Lr = p_.lr;

  const double sum_BxFx_0 = cd * fx[0] + cd * fx[1] + fx[2] + fx[3];
  const double sum_BxFx_1 = sd * fx[0] + sd * fx[1];
  const double sum_BxFx_2 = (Lf * sd - 0.5 * b * cd) * fx[0] + (Lf * sd + 0.5 * b * cd) * fx[1] + (-b) * fx[2] + (b) * fx[3];

  const double sum_ByFy_0 = (-sd) * fy[0] + (-sd) * fy[1];
  const double sum_ByFy_1 = (cd) * fy[0] + (cd) * fy[1] + fy[2] + fy[3];
  const double sum_ByFy_2 = (Lf * cd - 0.5 * b * sd) * fy[0] + (Lf * cd + 0.5 * b * sd) * fy[1] + (-Lr) * fy[2] + (-Lr) * fy[3];

  const double Fx_body = sum_BxFx_0 + sum_ByFy_0;
  const double Fy_body = sum_BxFx_1 + sum_ByFy_1;
  const double Mz = sum_BxFx_2 + sum_ByFy_2;

  // dv = M^-1 * ([Fx;Fy;Mz] - C)
  const double dvx = (Fx_body - Cx) / p_.m;
  const double dvy = (Fy_body - Cy) / p_.m;
  const double dr = Mz / p_.Izz;

  // Wheel angular accelerations:
  // dz3 = J^-1*(u(2:5)-0.1*F_x-0.01*z3)
  Vec<4> ddphi{};
  for (int i = 0; i < 4; ++i) {
    const double tau = u[static_cast<std::size_t>(1 + i)];
    const double damp = -0.1 * fx[static_cast<std::size_t>(i)] - 0.01 * x[static_cast<std::size_t>(6 + i)];
    ddphi[static_cast<std::size_t>(i)] = (tau + damp) / p_.wheel_J;
  }

  Vec<NX> dotz{dx, dy, dpsi, dvx, dvy, dr, ddphi[0], ddphi[1], ddphi[2], ddphi[3]};

  // MATLAB clamps dotz(4:6)
  dotz[3] = clampAbs(dotz[3], p_.accel_limit);
  dotz[4] = clampAbs(dotz[4], p_.accel_limit);
  dotz[5] = clampAbs(dotz[5], p_.accel_limit);

  return dotz;
}

Vec<FourWheelModel::NX> FourWheelModel::step(const Vec<NU>& u) {
  feasible_ = true;
  u_ = u;
  const auto dotz = dynamics(x_, u_);
  x_ = add(x_, scale(dotz, p_.dt));
  if (clamp_vx_nonnegative_ && x_[3] < 0.0) x_[3] = 0.0;
  feasible_ = checkFeasible(x_);
  last_dotz_ = dotz;
  return x_;
}

} // namespace ru_racer

