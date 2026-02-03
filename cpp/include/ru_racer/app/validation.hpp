#pragma once

#include "ru_racer/bike_model.hpp"
#include "ru_racer/config.hpp"

#include <filesystem>

namespace ru_racer { class FourWheelModel; }

namespace ru_racer::app {

// Runs a set of open-loop validation scenarios (straight accel/brake/constant steer/sine steer, etc.)
// and writes CSV logs into out_dir/validation/.
// Returns true on success.
bool runOpenLoopValidation(BikeModel& bike, const KeyValueConfig& cfg, const std::filesystem::path& out_dir);

// Same scenarios, but using the wheel-by-wheel FourWheelModel.
bool runOpenLoopValidationFourWheel(FourWheelModel& model, const KeyValueConfig& cfg, const std::filesystem::path& out_dir);

} // namespace ru_racer::app

