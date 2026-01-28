#include "ru_racer/nmpc.hpp"

#include "ru_racer/math.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ru_racer {

NmpcTracker::NmpcTracker(BikeModel model, NmpcParams p) : model_(std::move(model)), p_(p) {}

Vec<BikeModel::NU> NmpcTracker::projectU(const Vec<BikeModel::NU>& u, const NmpcParams& p) {
  Vec<BikeModel::NU> o = u;
  o[0] = sat(o[0], p.delta_max);
  o[1] = sat(o[1], p.sfx_max);
  return o;
}

double NmpcTracker::rolloutCostDeltaSeq(const Vec<BikeModel::NX>& x0,
                                        const std::vector<double>& deltas,
                                        double delta_prev,
                                        const std::vector<Vec<BikeModel::NX>>& ref_window) const {
  BikeModel m = model_;
  m.reset(x0, Vec<BikeModel::NU>{0.0, 0.0});

  double J = 0.0;
  const int N = std::min(static_cast<int>(ref_window.size()), p_.horizon_steps);
  for (int k = 0; k < N; ++k) {
    const auto& xr = ref_window[static_cast<std::size_t>(k)];

    const double delta = sat(deltas[static_cast<std::size_t>(std::min(k, static_cast<int>(deltas.size()) - 1))], p_.delta_max);
    const double dvx_now = (xr[3] - m.state()[3]);
    const double sfx = sat(p_.kv_vx * dvx_now, p_.sfx_max);
    const Vec<BikeModel::NU> u{delta, sfx};

    // One step forward, then penalize error at the next state.
    m.step(u);
    if (!m.isFeasible()) {
      J += 1e6;
      break;
    }

    const auto dx = sub(Vec<2>{m.state()[0], m.state()[1]}, Vec<2>{xr[0], xr[1]});
    const double dpsi = wrapAngle(m.state()[2] - xr[2]);
    const double dvx = m.state()[3] - xr[3];

    const double d_delta = (k == 0) ? (delta - delta_prev) : (delta - sat(deltas[static_cast<std::size_t>(k - 1)], p_.delta_max));
    J += p_.w_pos * dot(dx, dx) +
         p_.w_psi * (dpsi * dpsi) +
         p_.w_vx * (dvx * dvx) +
         p_.w_u * (delta * delta + sfx * sfx) +
         p_.w_delta_rate * (d_delta * d_delta);
  }
  return J;
}

std::vector<double> NmpcTracker::optimizeDeltaSeq(const Vec<BikeModel::NX>& x0,
                                                  const std::vector<double>& delta_init,
                                                  double delta_prev,
                                                  const std::vector<Vec<BikeModel::NX>>& ref_window) const {
  std::vector<double> deltas = delta_init;
  if (static_cast<int>(deltas.size()) != p_.horizon_steps) {
    deltas.assign(static_cast<std::size_t>(p_.horizon_steps), delta_prev);
  }
  for (auto& d : deltas) d = sat(d, p_.delta_max);

  for (int it = 0; it < p_.max_iters; ++it) {
    const double f0 = rolloutCostDeltaSeq(x0, deltas, delta_prev, ref_window);
    std::vector<double> grad(deltas.size(), 0.0);

    for (std::size_t i = 0; i < deltas.size(); ++i) {
      auto up = deltas;
      up[i] = sat(up[i] + p_.eps_fd, p_.delta_max);
      const double fp = rolloutCostDeltaSeq(x0, up, delta_prev, ref_window);
      grad[i] = (fp - f0) / p_.eps_fd;
    }

    auto cand = deltas;
    for (std::size_t i = 0; i < cand.size(); ++i) cand[i] = sat(cand[i] - p_.step_size * grad[i], p_.delta_max);
    const double f1 = rolloutCostDeltaSeq(x0, cand, delta_prev, ref_window);
    if (f1 < f0) {
      deltas = cand;
      continue;
    }
    // Backtrack.
    auto cand2 = deltas;
    for (std::size_t i = 0; i < cand2.size(); ++i) cand2[i] = sat(cand2[i] - 0.5 * p_.step_size * grad[i], p_.delta_max);
    if (rolloutCostDeltaSeq(x0, cand2, delta_prev, ref_window) < f0) deltas = cand2;
  }
  return deltas;
}

std::vector<NmpcLogStep> NmpcTracker::track(const std::vector<Vec<BikeModel::NX>>& ref) {
  std::vector<NmpcLogStep> log;
  if (ref.empty()) return log;

  BikeModel exec = model_;
  exec.reset(ref.front(), Vec<BikeModel::NU>{0.0, 0.0});

  Vec<BikeModel::NU> u{0.0, 0.0};
  double last_delta = 0.0;
  std::vector<double> delta_seq(static_cast<std::size_t>(p_.horizon_steps), 0.0);
  const int N = static_cast<int>(ref.size());
  log.reserve(ref.size());

  for (int k = 0; k < N; ++k) {
    if (k % p_.control_hold_steps == 0) {
      std::vector<Vec<BikeModel::NX>> win;
      win.reserve(static_cast<std::size_t>(p_.horizon_steps));
      for (int j = 0; j < p_.horizon_steps; ++j) {
        const int idx = std::min(N - 1, k + j);
        win.push_back(ref[static_cast<std::size_t>(idx)]);
      }

      // Shift warm-start sequence
      if (!delta_seq.empty()) {
        for (int i = 0; i + 1 < static_cast<int>(delta_seq.size()); ++i) delta_seq[static_cast<std::size_t>(i)] = delta_seq[static_cast<std::size_t>(i + 1)];
        delta_seq.back() = last_delta;
      }
      delta_seq = optimizeDeltaSeq(exec.state(), delta_seq, last_delta, win);
    }

    const double delta = sat(delta_seq.empty() ? last_delta : delta_seq.front(), p_.delta_max);
    const double dvx_now = ref[static_cast<std::size_t>(k)][3] - exec.state()[3];
    const double sfx = sat(p_.kv_vx * dvx_now, p_.sfx_max);
    u = Vec<BikeModel::NU>{delta, sfx};
    last_delta = delta;

    NmpcLogStep s{};
    s.t = k * exec.dt();
    s.x = exec.state();
    s.u = u;
    s.x_ref = ref[static_cast<std::size_t>(k)];
    log.push_back(s);

    // Step forward one dt with constant input.
    exec.step(u);
  }
  return log;
}

} // namespace ru_racer

