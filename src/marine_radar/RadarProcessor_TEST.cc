// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Unit tests of the complete core pipeline (RadarProcessor) driven by a
// small analytic ray caster (vertical cylinders) instead of Gazebo. This
// exercises scan scheduling, ray planning, echo formation, extraction,
// identification and the image end-to-end without a simulator.

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

#include "marine_radar/RadarProcessor.hh"

using namespace marine_radar;  // NOLINT

namespace
{

struct Cylinder
{
  double x, y, r, zmin, zmax;
};

/// Analytic ray cast against vertical cylinders (world frame).
RayResult Cast(const std::vector<Cylinder> &_world, const Pose &_pose,
               const RayRequest &_req)
{
  const Vec3 s = _pose.Transform(_req.start);
  const Vec3 e = _pose.Transform(_req.end);
  const Vec3 d = e - s;
  double best = std::numeric_limits<double>::infinity();
  const Cylinder *hitCyl = nullptr;
  for (const auto &c : _world)
  {
    const double fx = s.x - c.x, fy = s.y - c.y;
    const double a = d.x * d.x + d.y * d.y;
    const double b = 2 * (fx * d.x + fy * d.y);
    const double cc = fx * fx + fy * fy - c.r * c.r;
    const double disc = b * b - 4 * a * cc;
    if (a <= 0 || disc < 0)
      continue;
    const double t = (-b - std::sqrt(disc)) / (2 * a);
    if (t < 0 || t > 1)
      continue;
    const double z = s.z + t * d.z;
    if (z < c.zmin || z > c.zmax)
      continue;
    if (t < best)
    {
      best = t;
      hitCyl = &c;
    }
  }
  RayResult r;
  if (!hitCyl)
  {
    r.fraction = std::numeric_limits<double>::quiet_NaN();
    return r;
  }
  const Vec3 p = s + d * best;
  const Vec3 n((p.x - hitCyl->x) / hitCyl->r, (p.y - hitCyl->y) / hitCyl->r,
               0.0);
  r.fraction = best;
  r.point = _pose.InverseTransform(p);
  r.normal = _pose.orientation.RotateReverse(n);
  return r;
}

struct Sim
{
  std::vector<Cylinder> world;
  std::function<Pose(double)> poseAt = [](double)
  {
    Pose p;
    p.position = {0, 0, 5};
    return p;
  };
  std::vector<ScanResult> scans;
  uint64_t steps{0};

  /// Run from step _k0 to _k1 (1 ms steps).
  void Run(RadarProcessor &_p, uint64_t _k0, uint64_t _k1)
  {
    std::vector<RayResult> res;
    for (uint64_t k = _k0 + 1; k <= _k1; ++k)
    {
      const double t = static_cast<double>(k) * 1e-3;
      const Pose pose = poseAt(t);
      const auto &plan = _p.PlanRays(t, pose);
      res.clear();
      for (const auto &req : plan)
        res.push_back(Cast(world, pose, req));
      _p.SetRayResults(res, pose);
      for (auto &s : _p.Process(t, pose))
        scans.push_back(std::move(s));
      ++steps;
    }
  }
};

RadarConfig Quiet()
{
  RadarConfig c;
  c.intensity_noise_stddev_db = 0.0;
  return c;
}

const Target *Near(const ScanResult &_s, double _x, double _y, double _tol)
{
  for (const auto &t : _s.targets)
    if (std::hypot(t.position_world.x - _x, t.position_world.y - _y) <= _tol)
      return &t;
  return nullptr;
}

}  // namespace

TEST(RadarProcessor, RejectsInvalidConfig)
{
  RadarConfig c;
  c.range_max = 1.0;
  EXPECT_THROW(RadarProcessor p(c), std::invalid_argument);
}

TEST(RadarProcessor, ScanCadenceAndRayBudget)
{
  const RadarConfig c = Quiet();
  RadarProcessor p(c);
  Sim sim;
  sim.Run(p, 0, 5010);
  ASSERT_EQ(sim.scans.size(), 2u);
  EXPECT_NEAR(sim.scans[0].stamp, 2.5, 0.001);
  EXPECT_NEAR(sim.scans[1].stamp, 5.0, 0.001);
  EXPECT_EQ(sim.scans[1].scan_index, 1u);
  // One ray per height slice per spoke (plus beam look-ahead).
  const double perScan = static_cast<double>(c.NumSpokes()) *
                         static_cast<double>(c.slice_heights.size());
  EXPECT_NEAR(static_cast<double>(p.Stats().rays_planned), 2.0 * perScan,
              perScan * 0.02);
  // Every spoke due after the first update (t = 1 ms) and up to
  // t = 5.010 s was processed exactly once.
  EXPECT_EQ(p.Stats().spokes_processed,
            static_cast<uint64_t>(p.Scheduler().LastDueIndex(5.010) -
                                  p.Scheduler().LastDueIndex(0.001)));
  EXPECT_EQ(p.Stats().spokes_skipped, 0u);
}

TEST(RadarProcessor, SingleTargetAccuracy)
{
  const RadarConfig c = Quiet();
  RadarProcessor p(c);
  Sim sim;
  sim.world = std::vector<Cylinder>{{300.0, 100.0, 1.5, -1.0, 3.0}};
  sim.Run(p, 0, 5010);
  ASSERT_EQ(sim.scans.size(), 2u);
  ASSERT_EQ(sim.scans[1].targets.size(), 1u);
  const Target &t = sim.scans[1].targets[0];
  const double horiz = std::hypot(300.0, 100.0) - 1.5;
  EXPECT_NEAR(t.range, std::hypot(horiz, 5.0 - 1.5), 2.0);
  EXPECT_NEAR(RadToDeg(WrapAngle(t.bearing - std::atan2(100.0, 300.0))), 0.0,
              0.1);
  EXPECT_EQ(t.id, 1u);
  EXPECT_EQ(t.age, 2u);
  EXPECT_GT(t.intensity_db, c.detection_threshold_db);
  EXPECT_LE(t.time, sim.scans[1].stamp);
  EXPECT_GE(t.time, sim.scans[1].stamp - c.ScanPeriod());
  // Returns are consistent with the target.
  ASSERT_FALSE(sim.scans[1].returns.empty());
  for (const auto &r : sim.scans[1].returns)
    EXPECT_NEAR(r.range, t.range, 3.0);
}

TEST(RadarProcessor, RangeLimitControlsDetection)
{
  RadarConfig c = Quiet();
  Sim sim;
  sim.world = std::vector<Cylinder>{{1200.0, 0.0, 3.0, -1.0, 5.0}};
  {
    RadarProcessor p(c);
    sim.Run(p, 0, 2510);
    ASSERT_EQ(sim.scans.size(), 1u);
    EXPECT_TRUE(sim.scans[0].targets.empty());
    EXPECT_TRUE(sim.scans[0].returns.empty());
  }
  c.range_max = 1500.0;
  c.range_resolution = 3.0;
  sim.scans.clear();
  RadarProcessor p(c);
  sim.Run(p, 0, 2510);
  ASSERT_EQ(sim.scans.size(), 1u);
  EXPECT_EQ(sim.scans[0].targets.size(), 1u);
}

TEST(RadarProcessor, OcclusionByTallerObject)
{
  const RadarConfig c = Quiet();
  RadarProcessor p(c);
  Sim sim;
  sim.world = std::vector<Cylinder>{
               {200.0, 0.0, 10.0, -1.0, 30.0},   // tall, wide island
               {400.0, 0.0, 1.0, -1.0, 3.0},     // small buoy behind it
               {0.0, 400.0, 1.0, -1.0, 3.0}};    // control buoy
  sim.Run(p, 0, 2510);
  ASSERT_EQ(sim.scans.size(), 1u);
  EXPECT_NE(Near(sim.scans[0], 190.0, 0.0, 6.0), nullptr);
  EXPECT_EQ(Near(sim.scans[0], 400.0, 0.0, 8.0), nullptr);
  EXPECT_NE(Near(sim.scans[0], 0.0, 399.0, 6.0), nullptr);
}

TEST(RadarProcessor, MovingAndRotatingRadarKeepsWorldPositions)
{
  const RadarConfig c = Quiet();
  RadarProcessor p(c);
  Sim sim;
  // Both objects stay clear of the +-180 deg sweep seam while the vessel
  // turns (a target crossing the seam in the sweep direction can be swept
  // zero or two times in one revolution, as on a real radar).
  sim.world = std::vector<Cylinder>{{0.0, 300.0, 1.0, -1.0, 3.0},
                                    {-400.0, 150.0, 1.0, -1.0, 3.0}};
  sim.poseAt = [](double _t)
  {
    Pose pose;
    pose.position = {6.0 * _t, 0.0, 5.0};
    pose.orientation = Quat::FromYaw(0.15 * _t);
    return pose;
  };
  sim.Run(p, 0, 10010);
  ASSERT_EQ(sim.scans.size(), 4u);
  for (const auto &s : sim.scans)
  {
    const Target *a = Near(s, 0.0, 300.0, 3.0);
    const Target *b = Near(s, -400.0, 150.0, 3.0);
    EXPECT_NE(a, nullptr) << "scan " << s.scan_index;
    EXPECT_NE(b, nullptr) << "scan " << s.scan_index;
    EXPECT_EQ(s.targets.size(), 2u);
  }
  // Same objects keep their ids.
  const Target *first = Near(sim.scans[0], 0.0, 300.0, 3.0);
  const Target *last = Near(sim.scans[3], 0.0, 300.0, 3.0);
  ASSERT_TRUE(first && last);
  EXPECT_EQ(first->id, last->id);
}

TEST(RadarProcessor, SectorScanOnlySeesInsideFov)
{
  RadarConfig c = Quiet();
  c.horizontal_fov = DegToRad(90.0);
  RadarProcessor p(c);
  Sim sim;
  sim.world = std::vector<Cylinder>{{300.0 * std::cos(DegToRad(30.0)),
                300.0 * std::sin(DegToRad(30.0)), 1.0, -1.0, 3.0},
               {300.0 * std::cos(DegToRad(120.0)),
                300.0 * std::sin(DegToRad(120.0)), 1.0, -1.0, 3.0}};
  sim.Run(p, 0, 5010);
  ASSERT_EQ(sim.scans.size(), 2u);
  ASSERT_EQ(sim.scans[1].targets.size(), 1u);
  EXPECT_NEAR(RadToDeg(sim.scans[1].targets[0].bearing), 30.0, 0.15);
}

TEST(RadarProcessor, TimeJumpSkipsInsteadOfStalling)
{
  const RadarConfig c = Quiet();
  RadarProcessor p(c);
  Sim sim;
  sim.Run(p, 0, 1000);
  sim.Run(p, 100000, 100001);  // jump from 1 s to 100 s
  EXPECT_GT(p.Stats().spokes_skipped, 0u);
  EXPECT_LE(sim.scans.size(), 2u);
}

TEST(RadarProcessor, DeterministicWithSeedAndResettable)
{
  RadarConfig c;
  c.intensity_noise_stddev_db = 3.0;
  c.range_noise_stddev = 1.0;
  c.false_alarm_rate = 4.0;
  auto run = [&](const RadarConfig &_c)
  {
    RadarProcessor p(_c);
    Sim sim;
    sim.world = std::vector<Cylinder>{{250.0, -80.0, 2.0, -1.0, 4.0}};
    sim.Run(p, 0, 2510);
    return sim.scans;
  };
  const auto a = run(c);
  const auto b = run(c);
  ASSERT_EQ(a.size(), b.size());
  ASSERT_EQ(a[0].targets.size(), b[0].targets.size());
  for (size_t i = 0; i < a[0].targets.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(a[0].targets[i].range, b[0].targets[i].range);
    EXPECT_DOUBLE_EQ(a[0].targets[i].intensity_db,
                     b[0].targets[i].intensity_db);
  }
  c.seed = 1234;
  const auto d = run(c);
  bool differs = d[0].targets.size() != a[0].targets.size();
  for (size_t i = 0; !differs && i < a[0].targets.size(); ++i)
    differs = a[0].targets[i].intensity_db != d[0].targets[i].intensity_db;
  EXPECT_TRUE(differs);

  // Reset restarts time and ids.
  RadarProcessor p(Quiet());
  Sim sim;
  sim.world = std::vector<Cylinder>{{250.0, -80.0, 2.0, -1.0, 4.0}};
  sim.Run(p, 0, 2510);
  p.Reset();
  sim.scans.clear();
  sim.Run(p, 0, 2510);
  ASSERT_EQ(sim.scans.size(), 1u);
  ASSERT_EQ(sim.scans[0].targets.size(), 1u);
  EXPECT_EQ(sim.scans[0].targets[0].id, 1u);
  EXPECT_EQ(sim.scans[0].scan_index, 0u);
}

TEST(RadarProcessor, ImageAndDetectionsAreConsistent)
{
  RadarConfig c = Quiet();
  c.afterglow = 0.0;
  c.echo_expansion = false;
  RadarProcessor p(c);
  Sim sim;
  sim.world = std::vector<Cylinder>{{0.0, 600.0, 2.0, -1.0, 4.0},
                                    {-300.0, -300.0, 2.0, -1.0, 4.0}};
  sim.Run(p, 0, 2510);
  ASSERT_EQ(sim.scans.size(), 1u);
  ASSERT_EQ(p.LastTargets().size(), 2u);
  RgbImage img;
  Pose pose;
  pose.position = {0, 0, 5};
  p.RenderImage(2.51, pose, img);
  PpiRenderer r(c);
  for (const auto &t : p.LastTargets())
  {
    double u, v;
    r.ToPixel(std::hypot(t.position_radar.x, t.position_radar.y),
              r.DisplayAngle(t.bearing, 0.0), u, v);
    bool echo = false;
    for (int dv = -2; dv <= 2 && !echo; ++dv)
      for (int du = -2; du <= 2 && !echo; ++du)
      {
        const uint8_t *px = img.Pixel(static_cast<int>(std::lround(u)) + du,
                                      static_cast<int>(std::lround(v)) + dv);
        echo = px[1] >= 105 && px[2] <= 75;
      }
    EXPECT_TRUE(echo) << "target " << t.id << " missing from the image";
  }
  // Every echo cell of the scan is a return point.
  size_t cells = 0;
  for (const auto &t : sim.scans[0].targets)
    cells += t.num_cells;
  EXPECT_EQ(cells, sim.scans[0].returns.size());
}
