// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Unit tests: target extraction (clustering, wrap-around, properties,
// filtering) and target identification (persistent IDs).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "marine_radar/TargetExtraction.hh"

using namespace marine_radar;  // NOLINT

namespace
{
/// Small synthetic scan: 360 spokes of 1 deg, bins of 2 m from 10 m.
RadarConfig SmallConfig()
{
  RadarConfig c;
  c.angular_resolution = DegToRad(1.0);
  c.range_max = 500.0;
  return c;
}

std::vector<ProcessedSpoke> EmptyScan(const RadarConfig &_c, const Pose &_p)
{
  std::vector<ProcessedSpoke> scan(static_cast<size_t>(_c.NumSpokes()));
  for (int p = 0; p < _c.NumSpokes(); ++p)
  {
    auto &s = scan[static_cast<size_t>(p)];
    s.valid = true;
    s.g = p;
    s.azimuth_index = _c.NumSpokes() - 1 - p;  // clockwise sweep
    s.azimuth = _c.SpokeAzimuth(s.azimuth_index);
    s.time = p * 0.001;
    s.pose = _p;
  }
  return scan;
}

/// Add an echo at a given azimuth index and bin.
void AddEcho(const RadarConfig &_c, std::vector<ProcessedSpoke> &_scan,
             int _azIndex, int _bin, float _snr = 20.0f)
{
  const int p = _c.NumSpokes() - 1 - _azIndex;
  auto &e = _scan[static_cast<size_t>(p)].echoes;
  Echo echo;
  echo.bin = _bin;
  echo.snr_db = _snr;
  echo.range = static_cast<float>(_c.BinCenterRange(_bin));
  e.push_back(echo);
  std::sort(e.begin(), e.end(),
            [](const Echo &_a, const Echo &_b) { return _a.bin < _b.bin; });
}

int AzIndexOf(const RadarConfig &_c, double _deg)
{
  return static_cast<int>(std::floor((DegToRad(_deg) + _c.horizontal_fov / 2)
                                     / _c.SpokeSpacing()));
}
}  // namespace

TEST(TargetExtractor, BlobBecomesOneTarget)
{
  const RadarConfig c = SmallConfig();
  auto scan = EmptyScan(c, Pose());
  const int az = AzIndexOf(c, 40.0);  // spoke centred at 40.5 deg
  for (int da = -2; da <= 2; ++da)
    for (int b = 100; b <= 102; ++b)
      AddEcho(c, scan, az + da, b, da == 0 && b == 101 ? 30.0f : 20.0f);

  std::mt19937 rng(1);
  const auto targets = TargetExtractor(c).Extract(scan, rng);
  ASSERT_EQ(targets.size(), 1u);
  const Target &t = targets[0];
  EXPECT_EQ(t.num_cells, 15u);
  EXPECT_NEAR(RadToDeg(t.bearing), 40.5, 1e-6);
  EXPECT_NEAR(t.range, c.BinCenterRange(101), 1e-3);
  EXPECT_NEAR(t.min_range, c.BinCenterRange(100), 1e-3);
  EXPECT_NEAR(t.range_extent, 2 * 2.0 + 2.0, 1e-3);
  EXPECT_NEAR(RadToDeg(t.bearing_extent), 5.0, 1e-6);
  EXPECT_FLOAT_EQ(static_cast<float>(t.intensity_db), 30.0f);
  EXPECT_NEAR(t.position_radar.x, t.range * std::cos(t.bearing), 1e-9);
  EXPECT_NEAR(t.position_radar.y, t.range * std::sin(t.bearing), 1e-9);
}

TEST(TargetExtractor, SeparateBlobsAndGaps)
{
  RadarConfig c = SmallConfig();
  auto scan = EmptyScan(c, Pose());
  const int a = AzIndexOf(c, 10.0);
  AddEcho(c, scan, a, 50);
  AddEcho(c, scan, a + 1, 50);
  // Second object far in range on the same bearing.
  AddEcho(c, scan, a, 120);
  // Third object, one empty spoke away from the first: bridged by the
  // default cluster_spoke_gap = 1.
  AddEcho(c, scan, a + 3, 50);
  std::mt19937 rng(1);
  auto targets = TargetExtractor(c).Extract(scan, rng);
  EXPECT_EQ(targets.size(), 2u);
  c.cluster_spoke_gap = 0;
  targets = TargetExtractor(c).Extract(scan, rng);
  EXPECT_EQ(targets.size(), 3u);
}

TEST(TargetExtractor, WrapAroundOnFullCircle)
{
  RadarConfig c = SmallConfig();
  auto scan = EmptyScan(c, Pose());
  // Object straddling the sweep seam (azimuth +-180): sweep positions 0
  // and N-1 are azimuth indices N-1 and 0.
  AddEcho(c, scan, 0, 80);
  AddEcho(c, scan, c.NumSpokes() - 1, 80);
  std::mt19937 rng(1);
  auto targets = TargetExtractor(c).Extract(scan, rng);
  ASSERT_EQ(targets.size(), 1u);
  EXPECT_NEAR(std::abs(RadToDeg(targets[0].bearing)), 180.0, 1e-6);
  EXPECT_NEAR(RadToDeg(targets[0].bearing_extent), 2.0, 1e-6);
}

TEST(TargetExtractor, NoWrapForSector)
{
  RadarConfig c = SmallConfig();
  c.horizontal_fov = DegToRad(90.0);
  auto scan = EmptyScan(c, Pose());
  AddEcho(c, scan, 0, 80);
  AddEcho(c, scan, c.NumSpokes() - 1, 80);
  std::mt19937 rng(1);
  EXPECT_EQ(TargetExtractor(c).Extract(scan, rng).size(), 2u);
}

TEST(TargetExtractor, MinCellsAndFalseCells)
{
  RadarConfig c = SmallConfig();
  c.min_target_cells = 3;
  auto scan = EmptyScan(c, Pose());
  AddEcho(c, scan, 100, 30);   // single cell: filtered
  for (int b = 60; b < 64; ++b)
    AddEcho(c, scan, 200, b);
  scan[static_cast<size_t>(c.NumSpokes() - 1 - 200)].echoes[0].false_alarm =
      true;
  std::mt19937 rng(1);
  const auto t = TargetExtractor(c).Extract(scan, rng);
  ASSERT_EQ(t.size(), 1u);
  EXPECT_EQ(t[0].num_cells, 4u);
  EXPECT_EQ(t[0].num_false_cells, 1u);
}

TEST(TargetExtractor, WorldPositionUsesSpokePose)
{
  const RadarConfig c = SmallConfig();
  Pose pose;
  pose.position = {100.0, -50.0, 5.0};
  pose.orientation = Quat::FromYaw(DegToRad(30.0));
  auto scan = EmptyScan(c, pose);
  AddEcho(c, scan, AzIndexOf(c, 0.0), 145);  // 300 m, bearing 0.5 deg
  std::mt19937 rng(1);
  const auto t = TargetExtractor(c).Extract(scan, rng);
  ASSERT_EQ(t.size(), 1u);
  const Vec3 expected = pose.Transform(t[0].position_radar);
  EXPECT_NEAR(t[0].position_world.x, expected.x, 1e-9);
  EXPECT_NEAR(t[0].position_world.y, expected.y, 1e-9);
  // Range is measured from the radar: world distance matches.
  EXPECT_NEAR((t[0].position_world - pose.position).Length(), t[0].range,
              1e-6);
  const auto returns = TargetExtractor(c).Returns(scan);
  ASSERT_EQ(returns.size(), 1u);
  EXPECT_NEAR((returns[0].position_world - pose.position).Length(),
              returns[0].range, 1e-6);
}

TEST(TargetExtractor, BearingNoiseStatistics)
{
  RadarConfig c = SmallConfig();
  c.bearing_noise_stddev = DegToRad(0.5);
  auto scan = EmptyScan(c, Pose());
  AddEcho(c, scan, AzIndexOf(c, 0.0), 100);
  std::mt19937 rng(4);
  TargetExtractor ex(c);
  double sum = 0, sum2 = 0;
  const int n = 4000;
  for (int i = 0; i < n; ++i)
  {
    const double err = RadToDeg(ex.Extract(scan, rng)[0].bearing) - 0.5;
    sum += err;
    sum2 += err * err;
  }
  EXPECT_NEAR(sum / n, 0.0, 0.03);
  EXPECT_NEAR(std::sqrt(sum2 / n), 0.5, 0.03);
}

TEST(TargetTracker, PersistentIdsCoastingAndGate)
{
  RadarConfig c;
  c.association_gate = 20.0;
  c.track_coast_scans = 1;
  TargetTracker tracker(c);
  auto mk = [](double _x, double _y)
  {
    Target t;
    t.position_world = {_x, _y, 0};
    return t;
  };

  std::vector<Target> s1 = {mk(0, 0), mk(100, 0)};
  tracker.Associate(s1);
  EXPECT_EQ(s1[0].id, 1u);
  EXPECT_EQ(s1[1].id, 2u);
  EXPECT_EQ(s1[0].age, 1u);

  // Small motion: same ids, age 2. New object: id 3.
  std::vector<Target> s2 = {mk(105, 3), mk(4, -2), mk(500, 500)};
  tracker.Associate(s2);
  EXPECT_EQ(s2[0].id, 2u);
  EXPECT_EQ(s2[1].id, 1u);
  EXPECT_EQ(s2[2].id, 3u);
  EXPECT_EQ(s2[0].age, 2u);
  EXPECT_EQ(s2[2].age, 1u);

  // Object 1 missed for one scan (coasts), object 3 jumps beyond the gate.
  std::vector<Target> s3 = {mk(110, 6), mk(530, 500)};
  tracker.Associate(s3);
  EXPECT_EQ(s3[0].id, 2u);
  EXPECT_EQ(s3[1].id, 4u);

  // Object 1 returns within the coast window: keeps id 1, age restarts.
  std::vector<Target> s4 = {mk(6, -2)};
  tracker.Associate(s4);
  EXPECT_EQ(s4[0].id, 1u);
  EXPECT_EQ(s4[0].age, 1u);

  // Two missed scans exceed the coast window: a new id.
  std::vector<Target> none;
  tracker.Associate(none);
  tracker.Associate(none);
  std::vector<Target> s5 = {mk(6, -2)};
  tracker.Associate(s5);
  EXPECT_GT(s5[0].id, 4u);

  tracker.Reset();
  std::vector<Target> s6 = {mk(0, 0)};
  tracker.Associate(s6);
  EXPECT_EQ(s6[0].id, 1u);
}
