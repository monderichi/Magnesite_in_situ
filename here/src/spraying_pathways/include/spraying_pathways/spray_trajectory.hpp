#pragma once
#include "spraying_pathways/types.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>

#include <vector>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <limits>

namespace sp {
namespace traj {

using Eigen::Vector2d;

// ---------- small helpers ----------
inline Vector2d toVector(const sp::Point2D& p){ return {p.x, p.y}; }
inline sp::Point2D toPoint2D(const Vector2d& v){ return {v.x(), v.y()}; }

inline Vector2d unit_vector(const sp::Point2D& a, const sp::Point2D& b){
  Vector2d v = toVector(b) - toVector(a);
  const double n = v.norm();
  if (n == 0.0) throw std::runtime_error("Zero-length vector");
  return v / n;
}

inline double accel_distance(double v_target, double a_max){
  return (v_target * v_target) / (2.0 * a_max);
}

inline sp::Point2D offset_point(const sp::Point2D& p, const Vector2d& dir, double dist){
  return toPoint2D(toVector(p) - dir * dist);
}

inline std::tuple<sp::Point2D, sp::Point2D, double>
compute_start_end(const std::vector<sp::Point2D>& spray_pts, double v_target, double a_max){
  const double d = accel_distance(v_target, a_max);
  const Vector2d dir = unit_vector(spray_pts.front(), spray_pts.back());
  sp::Point2D start = offset_point(spray_pts.front(),  dir, d);
  sp::Point2D end   = offset_point(spray_pts.back(),  -dir, d);
  return {start, end, d};
}

// ---------- cubic time-scaling along a line ----------
inline std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, double>
smooth_velocity_profile_exact_a(double v_target, double a_max, int steps = 100){
  const double T = 1.5 * v_target / a_max;
  std::vector<double> t(steps), v(steps), a(steps);
  for (int i = 0; i < steps; ++i){
    t[i] = (T * i) / (steps - 1);
    const double tau = t[i] / T;
    v[i] = v_target * (3 * tau * tau - 2 * tau * tau * tau);
    a[i] = v_target * (6 * tau - 6 * tau * tau) / T;
  }
  return {t, v, a, T};
}

inline std::tuple<std::vector<double>, std::vector<sp::Point2D>, std::vector<double>, std::vector<double>,
                  sp::Point2D, std::vector<sp::Point2D>, sp::Point2D>
generate_trajectory(const std::vector<sp::Point2D>& spray_pts, double v_target, double a_max, int steps = 100)
{
  auto [start, end, d] = compute_start_end(spray_pts, v_target, a_max);
  const Vector2d dir_vec = unit_vector(spray_pts.front(), spray_pts.back());
  const double L = (toVector(spray_pts.back()) - toVector(spray_pts.front())).norm();

  // accel
  auto [t1, v1, a1, T1] = smooth_velocity_profile_exact_a(v_target, a_max, steps);
  std::vector<double> x1(steps);
  for (int i = 0; i < steps; ++i) x1[i] = (i==0 ? 0 : x1[i-1]) + v1[i]*(T1/steps);
  const double scale1 = d / x1.back();
  for (auto& x : x1) x *= scale1;

  // cruise
  const double T2 = L / v_target;
  std::vector<double> t2(steps), x2(steps), v2(steps, v_target), a2(steps, 0.0);
  for (int i = 0; i < steps; ++i){ t2[i] = T2 * i / (steps - 1); x2[i] = x1.back() + (L * i) / (steps - 1); }

  // decel
  auto [t3, v3, a3, T3] = smooth_velocity_profile_exact_a(v_target, a_max, steps);
  std::vector<double> x3(steps);
  for (int i = 0; i < steps; ++i) x3[i] = (i==0 ? 0 : x3[i-1]) + v3[steps-1-i]*(T3/steps);
  const double scale3 = d / x3.back();
  for (auto& x : x3) x = x2.back() + x*scale3;

  // concat time
  std::vector<double> t_all; t_all.insert(t_all.end(), t1.begin(), t1.end());
  for (auto& tv : t2) t_all.push_back(t1.back() + tv);
  for (auto& tv : t3) t_all.push_back(t1.back() + T2 + tv);

  // concat position
  std::vector<double> x_all; x_all.insert(x_all.end(), x1.begin(), x1.end());
  x_all.insert(x_all.end(), x2.begin(), x2.end());
  x_all.insert(x_all.end(), x3.begin(), x3.end());

  // velocities & accels
  std::vector<double> v_all; v_all.insert(v_all.end(), v1.begin(), v1.end());
  v_all.insert(v_all.end(), v2.begin(), v2.end());
  for (int i = steps-1; i>=0; --i) v_all.push_back(v3[i]);

  std::vector<double> a_all; a_all.insert(a_all.end(), a1.begin(), a1.end());
  a_all.insert(a_all.end(), a2.begin(), a2.end());
  for (int i = steps-1; i>=0; --i) a_all.push_back(-a3[i]);

  // build xy
  std::vector<sp::Point2D> xy; xy.reserve(x_all.size());
  const Vector2d origin = toVector(start);
  for (double xi : x_all) xy.emplace_back(toPoint2D(origin + dir_vec * xi));

  return {t_all, xy, v_all, a_all, start, spray_pts, end};
}

// ---------- smooth (quintic) transition between two points ----------
inline std::tuple<std::vector<double>, std::vector<sp::Point2D>, std::vector<double>, std::vector<double>>
smooth_cartesian_transition(const sp::Point2D& p0, const sp::Point2D& p1,
                            double a_max, int steps = 100, double tol = 1e-4)
{
  const double d = (toVector(p1) - toVector(p0)).norm();
  const Vector2d dir = unit_vector(p0, p1);

  auto quintic = [](double t, double T){
    const double tau = t / T;
    const double pos = 10*std::pow(tau,3) - 15*std::pow(tau,4) + 6*std::pow(tau,5);
    const double vel = (30*std::pow(tau,2) - 60*std::pow(tau,3) + 30*std::pow(tau,4)) / T;
    const double acc = (60*tau - 180*std::pow(tau,2) + 120*std::pow(tau,3)) / (T*T);
    return std::tuple<double,double,double>{pos,vel,acc};
  };

  double T_min = std::sqrt(15 * d / (8 * a_max)) * 0.5;
  double T_max = std::sqrt(15 * d / (8 * a_max)) * 2.0;

  while (T_max - T_min > tol) {
    const double T = 0.5 * (T_min + T_max);
    bool ok = true;
    for (int i = 0; i < steps; ++i) {
      const double ti = T * i / (steps - 1);
      auto [_, __, acc] = quintic(ti, T);
      if (std::abs(acc * d) > a_max) { ok = false; break; }
    }
    if (ok) T_max = 0.5 * (T_min + T_max);
    else    T_min = 0.5 * (T_min + T_max);
  }

  const double T = T_max;
  std::vector<double> t(steps), v(steps), a(steps);
  std::vector<sp::Point2D> xy; xy.reserve(steps);

  for (int i = 0; i < steps; ++i){
    t[i] = T * i / (steps - 1);
    auto [pos, vel, acc] = quintic(t[i], T);
    const double x = pos * d;
    xy.emplace_back(toPoint2D(toVector(p0) + dir * x));
    v[i] = vel * d;
    a[i] = acc * d;
  }
  return {t, xy, v, a};
}

// ---------- Bézier helpers ----------
inline Vector2d bezier_eval(double u, const Vector2d& P0, const Vector2d& P1,
                            const Vector2d& P2, const Vector2d& P3)
{
  const double B0 = std::pow(1-u,3);
  const double B1 = 3 * std::pow(1-u,2) * u;
  const double B2 = 3 * (1-u) * u * u;
  const double B3 = u*u*u;
  return B0*P0 + B1*P1 + B2*P2 + B3*P3;
}

inline Vector2d bezier_d1(double u, const Vector2d& P0, const Vector2d& P1,
                          const Vector2d& P2, const Vector2d& P3)
{
  const double dB0 = -3 * std::pow(1-u,2);
  const double dB1 =  3 * std::pow(1-u,2) - 6*(1-u)*u;
  const double dB2 =  6*(1-u)*u - 3*u*u;
  const double dB3 =  3*u*u;
  return dB0*P0 + dB1*P1 + dB2*P2 + dB3*P3;
}

inline void bezier_arclen_table(const Vector2d& P0, const Vector2d& P1,
                                const Vector2d& P2, const Vector2d& P3,
                                int n, std::vector<double>& u_tab, std::vector<double>& s_tab)
{
  u_tab.resize(n); s_tab.resize(n);
  u_tab[0] = 0.0; s_tab[0] = 0.0;
  for (int i = 1; i < n; ++i) {
    const double u0 = static_cast<double>(i-1) / (n-1);
    const double u1 = static_cast<double>(i)   / (n-1);
    u_tab[i] = u1;
    const Vector2d d0 = bezier_d1(u0, P0, P1, P2, P3);
    const Vector2d d1 = bezier_d1(u1, P0, P1, P2, P3);
    const double s_inc = 0.5 * (d0.norm() + d1.norm()) * (u1 - u0);
    s_tab[i] = s_tab[i-1] + s_inc;
  }
}

inline double interp1(const std::vector<double>& x, const std::vector<double>& y, double xq){
  if (xq <= x.front()) return y.front();
  if (xq >= x.back())  return y.back();
  auto it = std::lower_bound(x.begin(), x.end(), xq);
  const size_t i1 = static_cast<size_t>(std::distance(x.begin(), it));
  const size_t i0 = i1 - 1;
  const double t = (xq - x[i0]) / (x[i1] - x[i0] + 1e-15);
  return y[i0] + t * (y[i1] - y[i0]);
}

inline double invert_arclen_to_u(const std::vector<double>& u_tab,
                                 const std::vector<double>& s_tab, double s_query)
{
  return interp1(s_tab, u_tab, s_query);
}

/** Smooth Bézier change segment between two lines with speed shaping. */
inline std::tuple<std::vector<double>, std::vector<sp::Point2D>, std::vector<double>, std::vector<double>>
bezier_change_segment(const sp::Point2D& p_start, const Vector2d& dir_start,
                      const sp::Point2D& p_end,   const Vector2d& dir_end,
                      double v_const, double a_max,
                      int steps = 200,
                      double handle_len = 0.1,
                      double h_min_factor = 0.02,
                      double v_min_floor = 1e-4)
{
  const Vector2d P0 = toVector(p_start);
  const Vector2d P3 = toVector(p_end);
  Vector2d u0 = dir_start; if (u0.norm() > 0) u0.normalize();
  Vector2d u1 = dir_end;   if (u1.norm() > 0) u1.normalize();

  const double chord = (P3 - P0).norm();
  double h = handle_len;
  if (!std::isfinite(h)) h = std::max(h_min_factor * std::max(chord, 1e-9), 1e-9);

  const Vector2d P1 = P0 + h * u0;
  const Vector2d P2 = P3 - h * u1;

  const int n = std::max(3 * steps, 1000);
  std::vector<double> u_tab, s_tab; bezier_arclen_table(P0, P1, P2, P3, n, u_tab, s_tab);
  const double L = s_tab.back();

  std::vector<double> t(steps), v(steps), a(steps);
  std::vector<sp::Point2D> XY(steps);

  if (L <= 1e-12) {
    for (int i = 0; i < steps; ++i) { t[i] = (i==0?0.0:t[i-1]); v[i]=0.0; a[i]=0.0; XY[i] = {P0.x(), P0.y()}; }
    return {t, XY, v, a};
  }

  const double v_safe = std::max(v_const, 1e-12);
  const double T = L / v_safe;
  for (int i = 0; i < steps; ++i) t[i] = T * (static_cast<double>(i) / (steps - 1));

  const double a_allow = 3.0 * std::max(v_const - v_min_floor, 0.0) * v_const / std::max(L, 1e-12);
  const double a_eff   = std::min(a_max, a_allow);

  const double mid = T * 0.5;
  for (int i = 0; i < steps; ++i) {
    const double ti = t[i];
    if (ti <= mid) {
      const double xi = (2.0 / T) * ti;
      a[i] = -a_eff * 4.0 * xi * (1.0 - xi);
    } else {
      const double eta = (2.0 / T) * (ti - mid);
      a[i] =  a_eff * 4.0 * eta * (1.0 - eta);
    }
  }

  v[0] = v_const;
  for (int i = 1; i < steps; ++i) {
    const double dt = t[i] - t[i-1];
    v[i] = v[i-1] + 0.5 * (a[i-1] + a[i]) * dt;
    if (v[i] < v_min_floor) v[i] = v_min_floor;
  }

  std::vector<double> s(steps, 0.0);
  for (int i = 1; i < steps; ++i) {
    const double dt = t[i] - t[i-1];
    s[i] = s[i-1] + 0.5 * (v[i-1] + v[i]) * dt;
    if (s[i] < s[i-1]) s[i] = s[i-1];
  }

  const double s_end = s.back();
  if (s_end > 0) {
    const double scale = L / s_end;
    for (auto& si : s) si *= scale;
  }

  for (int i = 0; i < steps; ++i) {
    const double ui = invert_arclen_to_u(u_tab, s_tab, s[i]);
    const Vector2d Pi = bezier_eval(ui, P0, P1, P2, P3);
    XY[i] = {Pi.x(), Pi.y()};
  }

  return {t, XY, v, a};
}

// ---------- Multi-line stitchers ----------
/** V2-style: quintic transition segments between lines. */
inline std::tuple<std::vector<double>, std::vector<sp::Point2D>, std::vector<double>, std::vector<double>>
generate_multi_line_trajectory_quintic(const std::vector<std::vector<sp::Point2D>>& lines,
                                       double v_target, double a_max, int steps = 100)
{
  std::vector<double> t_all, v_all, a_all;
  std::vector<sp::Point2D> xy_all;
  double t_offset = 0.0;

  for (size_t i = 0; i < lines.size(); ++i) {
    auto [t, xy, v, a, start, pts, end] = generate_trajectory(lines[i], v_target, a_max, steps);
    for (auto& ti : t) t_all.push_back(ti + t_offset);
    xy_all.insert(xy_all.end(), xy.begin(), xy.end());
    v_all.insert(v_all.end(), v.begin(), v.end());
    a_all.insert(a_all.end(), a.begin(), a.end());
    t_offset = t_all.back();

    if (i + 1 < lines.size()) {
      auto [next_start, __, ___] = compute_start_end(lines[i+1], v_target, a_max);
      auto [tt, xyt, vt, at] = smooth_cartesian_transition(end, next_start, a_max, steps);
      for (auto& ti : tt) t_all.push_back(ti + t_offset);
      xy_all.insert(xy_all.end(), xyt.begin(), xyt.end());
      v_all.insert(v_all.end(), vt.begin(), vt.end());
      a_all.insert(a_all.end(), at.begin(), at.end());
      t_offset = t_all.back();
    }
  }
  return {t_all, xy_all, v_all, a_all};
}

/** V3-style: Bézier change segments between lines; keeps accel+cruise for first & decel for last. */
inline std::tuple<std::vector<double>, std::vector<sp::Point2D>, std::vector<double>, std::vector<double>>
generate_multi_line_trajectory_bezier(const std::vector<std::vector<sp::Point2D>>& lines,
                                      double v_target, double a_max, int steps = 100)
{
  std::vector<double> t_all, v_all, a_all;
  std::vector<sp::Point2D> xy_all;
  double t_offset = 0.0;

  for (size_t i = 0; i < lines.size(); ++i) {
    auto [t_line, xy_line, v_line, a_line, start, spray_pts, end] =
        generate_trajectory(lines[i], v_target, a_max, steps);

    const int i_acc_end    = steps - 1;
    const int i_cruise_beg = steps;
    const int i_cruise_end = 2 * steps - 1;
    const int i_decel_beg  = 2 * steps;
    const int i_last       = 3 * steps - 1;

    const int sl_beg = (i == 0) ? 0 : i_cruise_beg;
    const int sl_end = i_cruise_end;

    const double t0 = t_line[sl_beg];
    for (int k = sl_beg; k <= sl_end; ++k) {
      t_all.push_back((t_line[k] - t0) + t_offset);
      xy_all.push_back(xy_line[k]);
      v_all.push_back(v_line[k]);
      a_all.push_back(a_line[k]);
    }
    t_offset = t_all.back();

    if (i + 1 < lines.size()) {
      const auto& cur = lines[i];
      const auto& nxt = lines[i + 1];
      const sp::Point2D p_start = cur.back();
      const sp::Point2D p_end   = nxt.front();
      const Vector2d dir_start = unit_vector(cur.front(), cur.back());
      const Vector2d dir_end   = unit_vector(nxt.front(), nxt.back());

      auto [t_tr, xy_tr, v_tr, a_tr] = bezier_change_segment(
          p_start, dir_start, p_end, dir_end,
          v_target, a_max, steps, 0.2, 0.02, 1e-4);

      const double t0tr = t_tr.front();
      for (size_t k = 0; k < t_tr.size(); ++k) {
        t_all.push_back((t_tr[k] - t0tr) + t_offset);
        xy_all.push_back(xy_tr[k]);
        v_all.push_back(v_tr[k]);
        a_all.push_back(a_tr[k]);
      }
      t_offset = t_all.back();
    } else {
      const double t0dec = t_line[i_decel_beg];
      for (int k = i_decel_beg; k <= i_last; ++k) {
        t_all.push_back((t_line[k] - t0dec) + t_offset);
        xy_all.push_back(xy_line[k]);
        v_all.push_back(v_line[k]);
        a_all.push_back(a_line[k]);
      }
      t_offset = t_all.back();
    }
  }
  return {t_all, xy_all, v_all, a_all};
}

// ---------- nearest-time query ----------
inline double nearest_time_at_xy(const std::vector<sp::Point2D>& xy_traj,
                                 const std::vector<double>& t_traj,
                                 const sp::Point2D& query_xy)
{
  double best = std::numeric_limits<double>::infinity();
  size_t idx  = 0;
  for (size_t i = 0; i < xy_traj.size(); ++i) {
    const double dx = xy_traj[i].x - query_xy.x;
    const double dy = xy_traj[i].y - query_xy.y;
    const double d2 = dx*dx + dy*dy;
    if (d2 < best) { best = d2; idx = i; }
  }
  return t_traj[idx];
}

// ---------- organize poses into zig-zag spray lines ----------
inline std::vector<std::vector<sp::Point2D>>
organize_waypoints_into_spray_lines(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                                    double y_tolerance = 1e-4)
{
  std::vector<std::vector<sp::Point2D>> lines;

  for (const auto& pose : waypoints) {
    sp::Point2D pt{pose.position.x, pose.position.y};
    bool added = false;
    for (auto& line : lines) {
      if (!line.empty() && std::abs(line.front().y - pt.y) < y_tolerance) {
        line.push_back(pt); added = true; break;
      }
    }
    if (!added) lines.push_back({pt});
  }

  std::sort(lines.begin(), lines.end(),
            [](const auto& a, const auto& b){ return a.front().y > b.front().y; });

  for (size_t i = 0; i < lines.size(); ++i) {
    auto& line = lines[i];
    std::sort(line.begin(), line.end(),
              [](const sp::Point2D& a, const sp::Point2D& b){ return a.x < b.x; });
    if (i % 2 == 1) std::reverse(line.begin(), line.end());
  }
  return lines;
}

} // namespace traj
} // namespace sp
