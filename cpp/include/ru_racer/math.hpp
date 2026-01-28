#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>

namespace ru_racer {

template <std::size_t N>
using Vec = std::array<double, N>;

template <std::size_t N>
inline Vec<N> vzeros() {
  Vec<N> v{};
  v.fill(0.0);
  return v;
}

template <std::size_t N>
inline double dot(const Vec<N>& a, const Vec<N>& b) {
  double s = 0.0;
  for (std::size_t i = 0; i < N; ++i) s += a[i] * b[i];
  return s;
}

template <std::size_t N>
inline double norm2(const Vec<N>& a) {
  return std::sqrt(dot(a, a));
}

template <std::size_t N>
inline Vec<N> add(const Vec<N>& a, const Vec<N>& b) {
  Vec<N> o{};
  for (std::size_t i = 0; i < N; ++i) o[i] = a[i] + b[i];
  return o;
}

template <std::size_t N>
inline Vec<N> sub(const Vec<N>& a, const Vec<N>& b) {
  Vec<N> o{};
  for (std::size_t i = 0; i < N; ++i) o[i] = a[i] - b[i];
  return o;
}

template <std::size_t N>
inline Vec<N> scale(const Vec<N>& a, double s) {
  Vec<N> o{};
  for (std::size_t i = 0; i < N; ++i) o[i] = a[i] * s;
  return o;
}

inline double wrapAngle(double x) {
  // Wrap to [-pi, pi]
  constexpr double twoPi = 2.0 * M_PI;
  x = std::fmod(x + M_PI, twoPi);
  if (x < 0) x += twoPi;
  return x - M_PI;
}

inline double sat(double x, double s) {
  if (s < 0) throw std::invalid_argument("sat: s must be >= 0");
  if (x > s) return s;
  if (x < -s) return -s;
  return x;
}

} // namespace ru_racer

