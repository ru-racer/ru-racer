#pragma once

#include <filesystem>
#include <fstream>
#include <cctype>
#include <sstream>
#include <string>
#include <unordered_map>

namespace ru_racer {

// Simple key=value config. Lines starting with # are comments.
// Example:
//   start_x=0
//   max_nodes=50000
//   obstacles_csv=../data/obstacles.csv
class KeyValueConfig {
 public:
  bool load(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in.good()) return false;
    kv_.clear();
    std::string line;
    while (std::getline(in, line)) {
      auto trimmed = trim(line);
      if (trimmed.empty() || trimmed[0] == '#') continue;
      auto eq = trimmed.find('=');
      if (eq == std::string::npos) continue;
      auto k = trim(trimmed.substr(0, eq));
      auto v = trim(trimmed.substr(eq + 1));
      kv_[k] = v;
    }
    return true;
  }

  bool has(const std::string& k) const { return kv_.find(k) != kv_.end(); }

  std::string getString(const std::string& k, const std::string& def = "") const {
    auto it = kv_.find(k);
    return it == kv_.end() ? def : it->second;
  }

  int getInt(const std::string& k, int def) const {
    auto it = kv_.find(k);
    if (it == kv_.end()) return def;
    return std::stoi(it->second);
  }

  double getDouble(const std::string& k, double def) const {
    auto it = kv_.find(k);
    if (it == kv_.end()) return def;
    return std::stod(it->second);
  }

  bool getBool(const std::string& k, bool def) const {
    auto it = kv_.find(k);
    if (it == kv_.end()) return def;
    auto v = it->second;
    for (auto& c : v) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (v == "1" || v == "true" || v == "yes" || v == "on") return true;
    if (v == "0" || v == "false" || v == "no" || v == "off") return false;
    return def;
  }

 private:
  static std::string trim(std::string s) {
    auto isSpace = [](unsigned char c) { return std::isspace(c) != 0; };
    while (!s.empty() && isSpace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
    while (!s.empty() && isSpace(static_cast<unsigned char>(s.back()))) s.pop_back();
    return s;
  }

  std::unordered_map<std::string, std::string> kv_;
};

} // namespace ru_racer

