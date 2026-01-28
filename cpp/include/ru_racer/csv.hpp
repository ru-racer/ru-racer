#pragma once

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace ru_racer {

class CsvWriter {
 public:
  explicit CsvWriter(std::filesystem::path path) : path_(std::move(path)) {
    std::filesystem::create_directories(path_.parent_path());
    out_.open(path_, std::ios::out | std::ios::trunc);
  }

  bool good() const { return out_.good(); }
  const std::filesystem::path& path() const { return path_; }

  void writeRow(const std::vector<std::string>& cols) {
    for (std::size_t i = 0; i < cols.size(); ++i) {
      if (i) out_ << ",";
      out_ << escape(cols[i]);
    }
    out_ << "\n";
  }

  template <typename... Ts>
  std::enable_if_t<
      !(sizeof...(Ts) == 1 &&
        (std::is_same_v<std::decay_t<Ts>, std::vector<std::string>> && ...)),
      void>
  writeRow(Ts&&... cols) {
    std::vector<std::string> row{toString(std::forward<Ts>(cols))...};
    writeRow(row);
  }

 private:
  static std::string escape(const std::string& s) {
    bool needsQuotes = false;
    for (char c : s) {
      if (c == '"' || c == ',' || c == '\n' || c == '\r') {
        needsQuotes = true;
        break;
      }
    }
    if (!needsQuotes) return s;
    std::string o;
    o.reserve(s.size() + 2);
    o.push_back('"');
    for (char c : s) {
      if (c == '"') o.push_back('"');
      o.push_back(c);
    }
    o.push_back('"');
    return o;
  }

  template <typename T>
  static std::string toString(T&& v) {
    using U = std::decay_t<T>;
    if constexpr (std::is_same_v<U, std::string>) return v;
    if constexpr (std::is_same_v<U, const char*>) return std::string(v);
    if constexpr (std::is_same_v<U, std::string_view>) return std::string(v);
    if constexpr (std::is_arithmetic_v<U>) return std::to_string(static_cast<long double>(v));
    if constexpr (std::is_convertible_v<U, std::string_view>) return std::string(std::string_view(v));
    std::ostringstream oss;
    oss << v;
    return oss.str();
  }

  std::filesystem::path path_;
  std::ofstream out_;
};

} // namespace ru_racer

