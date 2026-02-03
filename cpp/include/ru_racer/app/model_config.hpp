#pragma once

#include "ru_racer/bike_model.hpp"
#include "ru_racer/config.hpp"
#include "ru_racer/four_wheel_model.hpp"

namespace ru_racer::app {

inline BikeParams loadBikeParamsFromConfig(const KeyValueConfig& cfg) {
  BikeParams bp{};

  // Integration-ish parameters
  bp.dt = cfg.getDouble("dt", bp.dt);
  bp.rewiring_value = cfg.getDouble("rewiring_value", bp.rewiring_value);

  // Vehicle constants (rarely change)
  bp.m = cfg.getDouble("vehicle_m", bp.m);
  bp.Izz = cfg.getDouble("vehicle_Izz", bp.Izz);
  bp.lf = cfg.getDouble("vehicle_lf", bp.lf);
  bp.lr = cfg.getDouble("vehicle_lr", bp.lr);
  bp.h = cfg.getDouble("vehicle_h", bp.h);
  bp.g = cfg.getDouble("vehicle_g", bp.g);

  // Tire (Magic Formula) nominal parameters
  bp.tiref.B = cfg.getDouble("tiref_B", bp.tiref.B);
  bp.tiref.C = cfg.getDouble("tiref_C", bp.tiref.C);
  bp.tiref.D = cfg.getDouble("tiref_D", bp.tiref.D);

  bp.tirer.B = cfg.getDouble("tirer_B", bp.tirer.B);
  bp.tirer.C = cfg.getDouble("tirer_C", bp.tirer.C);
  bp.tirer.D = cfg.getDouble("tirer_D", bp.tirer.D);

  return bp;
}

inline FrictionConfig loadFrictionConfigFromConfig(const KeyValueConfig& cfg) {
  FrictionConfig fc{};
  fc.mu_scale_f0 = cfg.getDouble("friction_mu_scale_f0", fc.mu_scale_f0);
  fc.mu_scale_r0 = cfg.getDouble("friction_mu_scale_r0", fc.mu_scale_r0);
  fc.mu_scale_f_min = cfg.getDouble("friction_mu_scale_f_min", fc.mu_scale_f_min);
  fc.mu_scale_f_max = cfg.getDouble("friction_mu_scale_f_max", fc.mu_scale_f_max);
  fc.mu_scale_r_min = cfg.getDouble("friction_mu_scale_r_min", fc.mu_scale_r_min);
  fc.mu_scale_r_max = cfg.getDouble("friction_mu_scale_r_max", fc.mu_scale_r_max);
  fc.mu_speed_dependent = cfg.getBool("friction_mu_speed_dependent", fc.mu_speed_dependent);
  fc.mu_speed_vscale_mps = cfg.getDouble("friction_mu_speed_vscale_mps", fc.mu_speed_vscale_mps);
  return fc;
}

inline void configureBikeRuntimeParams(BikeModel& bike, const KeyValueConfig& cfg) {
  // Friction (online-updatable): set initial estimate + bounds.
  bike.setFrictionConfig(loadFrictionConfigFromConfig(cfg));
  bike.setClampVxNonnegative(cfg.getBool("model_clamp_vx_nonnegative", false));
}

inline FourWheelParams loadFourWheelParamsFromConfig(const KeyValueConfig& cfg) {
  FourWheelParams p{};
  p.dt = cfg.getDouble("dt", p.dt);
  p.g = cfg.getDouble("vehicle_g", p.g);

  p.m = cfg.getDouble("vehicle_m", p.m);
  p.Izz = cfg.getDouble("vehicle_Izz", p.Izz);
  p.lf = cfg.getDouble("vehicle_lf", p.lf);
  p.lr = cfg.getDouble("vehicle_lr", p.lr);
  p.h = cfg.getDouble("vehicle_h", p.h);
  p.b = cfg.getDouble("vehicle_track_b", p.b);

  p.tire_r = cfg.getDouble("tire_r", p.tire_r);
  p.wheel_J = cfg.getDouble("wheel_J", p.wheel_J);

  p.tire_B = cfg.getDouble("tire_B", p.tire_B);
  p.tire_C = cfg.getDouble("tire_C", p.tire_C);
  p.tire_D = cfg.getDouble("tire_D", p.tire_D);
  p.tire_E = cfg.getDouble("tire_E", p.tire_E);

  p.accel_limit = cfg.getDouble("fourwheel_accel_limit", p.accel_limit);
  return p;
}

inline void configureFourWheelRuntimeParams(FourWheelModel& m, const KeyValueConfig& cfg) {
  m.setFrictionConfig(loadFrictionConfigFromConfig(cfg));
  m.setClampVxNonnegative(cfg.getBool("model_clamp_vx_nonnegative", false));
}

} // namespace ru_racer::app

