#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace ru_racer {

// Minimal occupancy grid supporting PGM (P2/P5).
// Convention:
// - stored values are 0..255
// - isFreePixel() returns true if value > free_threshold (default 0) unless invert=true
//
// World->pixel transform mimics MATLAB `Bike.check_feasibility_map`:
//   X = ceil(x*100 + 2.819*100)
//   Y = ceil(3.581*100 - y*100)
//
// That is: px = ceil(x*pixels_per_meter + origin_x_px)
//          py = ceil(origin_y_px - y*pixels_per_meter)
struct OccupancyGrid {
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> data; // row-major, size = width*height

  // Feasibility rule:
  // - invert=false => free if pixel_value > free_threshold  (matches typical binary maps where free=255, obstacle=0)
  // - invert=true  => free if pixel_value <= free_threshold (useful if you have inverse maps)
  bool invert = false;

  // Legacy MATLAB-like transform (used if use_affine_transform=false)
  double pixels_per_meter = 100.0;
  double origin_x_px = 2.819 * 100.0;
  double origin_y_px = 3.581 * 100.0;
  std::uint8_t free_threshold = 0;

  // General affine transform for world->pixel (optional).
  // Pixel coords are computed as:
  //   u = ceil(a11*x + a12*y + b1)
  //   v = ceil(a21*x + a22*y + b2)
  // Then converted to 0-based pixel indices via (u-1, v-1).
  //
  // Example to match MATLAB Implot2d overlay on track.jpg:
  //   u = 687 + 360*y
  //   v = 1125 + 390*x
  bool use_affine_transform = false;
  double a11 = 0.0, a12 = 0.0, a21 = 0.0, a22 = 0.0, b1 = 0.0, b2 = 0.0;

  bool loadPgm(const std::filesystem::path& path, std::string* err = nullptr);
  bool loadBmp(const std::filesystem::path& path, std::string* err = nullptr);

  // Convenience: choose loader based on extension (.pgm, .bmp).
  bool load(const std::filesystem::path& path, std::string* err = nullptr);

  bool inBounds(int px, int py) const { return px >= 0 && py >= 0 && px < width && py < height; }

  std::uint8_t at(int px, int py) const { return data[static_cast<std::size_t>(py) * static_cast<std::size_t>(width) + static_cast<std::size_t>(px)]; }

  bool isFreePixel(int px, int py) const {
    if (!inBounds(px, py)) return false;
    const auto v = at(px, py);
    if (!invert) return v > free_threshold;
    return v <= free_threshold;
  }

  bool isFreeWorld(double x_m, double y_m) const;
};

} // namespace ru_racer

