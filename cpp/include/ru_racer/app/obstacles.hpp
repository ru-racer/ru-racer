#pragma once

#include "ru_racer/bike_model.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace ru_racer::app {

inline bool loadCircleObstaclesCsv(const std::filesystem::path& path, std::vector<ru_racer::CircleObstacle>& out) {
  std::ifstream in(path);
  if (!in.good()) return false;
  out.clear();
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::stringstream ss(line);
    std::string a, b, c;
    if (!std::getline(ss, a, ',')) continue;
    if (!std::getline(ss, b, ',')) continue;
    if (!std::getline(ss, c, ',')) continue;
    ru_racer::CircleObstacle o{};
    o.x = std::stod(a);
    o.y = std::stod(b);
    o.r = std::stod(c);
    out.push_back(o);
  }
  return true;
}

} // namespace ru_racer::app

