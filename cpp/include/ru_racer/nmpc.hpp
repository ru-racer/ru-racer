#pragma once

#include "ru_racer/bike_model.hpp"

#include <vector>

namespace ru_racer {

struct NmpcParams {
  int horizon_steps = 15;
  int max_iters = 25;
  double step_size = 0.5;
  double eps_fd = 1e-3;

  int control_hold_steps = 10; // update u every PH steps, like MATLAB

  double w_pos = 1.0;
  double w_psi = 0.2;
  double w_vx = 0.2;
  double w_u = 0.01;
  double w_delta_rate = 0.2;

  // Practical tracker mode: optimize steering sequence only; compute longitudinal input to track vx_ref.
  bool optimize_delta_sequence = true;
  double kv_vx = 0.4; // sfx = sat(kv_vx*(vx_ref - vx), sfx_max)

  // Bounds on control
  double delta_max = M_PI / 9.0;
  double sfx_max = 0.1;
};

struct NmpcLogStep {
  double t = 0.0;
  Vec<BikeModel::NX> x{};
  Vec<BikeModel::NU> u{};
  Vec<BikeModel::NX> x_ref{};
};

class NmpcTracker {
 public:
  NmpcTracker(BikeModel model, NmpcParams p);

  // Track a provided reference trajectory (same dt as BikeModel).
  // Returns executed log at dt resolution.
  std::vector<NmpcLogStep> track(const std::vector<Vec<BikeModel::NX>>& ref);

 private:
  double rolloutCostDeltaSeq(const Vec<BikeModel::NX>& x0,
                             const std::vector<double>& deltas,
                             double delta_prev,
                             const std::vector<Vec<BikeModel::NX>>& ref_window) const;

  std::vector<double> optimizeDeltaSeq(const Vec<BikeModel::NX>& x0,
                                       const std::vector<double>& delta_init,
                                       double delta_prev,
                                       const std::vector<Vec<BikeModel::NX>>& ref_window) const;

  static Vec<BikeModel::NU> projectU(const Vec<BikeModel::NU>& u, const NmpcParams& p);

  BikeModel model_;
  NmpcParams p_{};
};

} // namespace ru_racer

