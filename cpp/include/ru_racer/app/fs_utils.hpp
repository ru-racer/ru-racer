#pragma once

#include <chrono>
#include <cstdio>
#include <string>

namespace ru_racer::app {

inline std::string nowStamp() {
  using clock = std::chrono::system_clock;
  const auto t = clock::to_time_t(clock::now());
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
                tm.tm_sec);
  return std::string(buf);
}

inline std::string shellQuote(const std::string& s) {
  // Safe for /bin/sh: wrap in single quotes, escape embedded single quotes.
  std::string out;
  out.reserve(s.size() + 2);
  out.push_back('\'');
  for (char c : s) {
    if (c == '\'') out += "'\"'\"'";
    else out.push_back(c);
  }
  out.push_back('\'');
  return out;
}

} // namespace ru_racer::app

