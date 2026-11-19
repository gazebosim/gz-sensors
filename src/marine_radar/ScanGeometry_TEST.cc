// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Unit tests: antenna scheduling (angular scan generation) and beam model.

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "marine_radar/RadarConfig.hh"
#include "marine_radar/ScanGeometry.hh"

using namespace marine_radar;  // NOLINT

TEST(AntennaScheduler, DueTimesAreMonotonicAndCoverOneRevolution)
{
  RadarConfig c;
  AntennaScheduler s(c);
  const int n = s.SpokesPerScan();
  for (int64_t g = 0; g < 3 * n; ++g)
    ASSERT_LT(s.DueTime(g), s.DueTime(g + 1));
  EXPECT_NEAR(s.DueTime(n) - s.DueTime(0), c.ScanPeriod(), 1e-9);
  EXPECT_NEAR(s.DueTime(1) - s.DueTime(0), s.Dwell(), 1e-12);
}

TEST(AntennaScheduler, LastDueIndexInvertsDueTime)
{
  RadarConfig c;
  c.angular_resolution = DegToRad(0.7);
  AntennaScheduler s(c);
  EXPECT_EQ(s.LastDueIndex(0.0), -1);
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> t(0.0, 50.0);
  for (int i = 0; i < 2000; ++i)
  {
    const double tt = t(rng);
    const int64_t g = s.LastDueIndex(tt);
    ASSERT_GE(g, 0);
    EXPECT_LE(s.DueTime(g), tt);
    EXPECT_GT(s.DueTime(g + 1), tt);
  }
  // Exactly at a due time.
  EXPECT_EQ(s.LastDueIndex(s.DueTime(1234)), 1234);
}

TEST(AntennaScheduler, ClockwiseRotation)
{
  RadarConfig c;
  AntennaScheduler s(c);
  // Azimuth decreases from one spoke to the next (clockwise from above).
  for (int64_t g = 0; g + 1 < s.SpokesPerScan(); ++g)
  {
    const double d = WrapAngle(s.SpokeAzimuth(g + 1) - s.SpokeAzimuth(g));
    ASSERT_NEAR(d, -c.SpokeSpacing(), 1e-9) << g;
  }
  // The beam points at a spoke exactly at its due time.
  for (int64_t g : {0, 17, 2048, 4095, 5000})
  {
    EXPECT_NEAR(WrapAngle(s.AntennaAzimuth(s.DueTime(g)) - s.SpokeAzimuth(g)),
                0.0, 1e-9) << g;
  }
  // Every azimuth index is visited exactly once per revolution.
  std::vector<int> seen(static_cast<size_t>(s.SpokesPerScan()), 0);
  for (int64_t g = 0; g < s.SpokesPerScan(); ++g)
    ++seen[static_cast<size_t>(s.AzimuthIndex(g))];
  for (int v : seen)
    ASSERT_EQ(v, 1);
}

TEST(AntennaScheduler, SectorScanHasBlankingGap)
{
  RadarConfig c;
  c.horizontal_fov = DegToRad(120.0);
  c.angular_resolution = DegToRad(1.0);
  AntennaScheduler s(c);
  const int n = s.SpokesPerScan();
  ASSERT_EQ(n, 120);
  // Spokes stay inside the sector.
  for (int64_t g = 0; g < 2 * n; ++g)
    ASSERT_LE(std::abs(s.SpokeAzimuth(g)), DegToRad(60.0) + 1e-9);
  // Inside the sector the spacing is one dwell; across the blanked sector
  // it is the remaining 240 degrees.
  EXPECT_NEAR(s.DueTime(1) - s.DueTime(0), s.Dwell(), 1e-12);
  EXPECT_NEAR(s.DueTime(n) - s.DueTime(n - 1),
              c.ScanPeriod() * (240.0 + 1.0) / 360.0, 1e-9);
  EXPECT_TRUE(s.Contiguous(5, 6));
  EXPECT_FALSE(s.Contiguous(n - 1, n));
  // Full-circle scans are contiguous across revolutions.
  RadarConfig full;
  AntennaScheduler sf(full);
  EXPECT_TRUE(sf.Contiguous(sf.SpokesPerScan() - 1, sf.SpokesPerScan()));
}

TEST(BeamModel, GainPattern)
{
  RadarConfig c;
  BeamModel b(c);
  EXPECT_DOUBLE_EQ(b.HorizontalGainDb(0.0), 0.0);
  EXPECT_NEAR(b.HorizontalGainDb(c.beamwidth_horizontal / 2), -3.0103, 1e-3);
  EXPECT_NEAR(b.HorizontalGainDb(-c.beamwidth_horizontal / 2), -3.0103, 1e-3);
  EXPECT_NEAR(b.HorizontalGainDb(c.beamwidth_horizontal), -12.041, 1e-3);
  EXPECT_NEAR(b.VerticalGainDb(c.beamwidth_vertical / 2), -3.0103, 1e-3);
  EXPECT_EQ(b.HalfBeamSpokes(),
            static_cast<int>(std::ceil(c.beamwidth_horizontal /
                                       c.SpokeSpacing() - 1e-9)));
  const Vec3 d = BeamModel::Direction(DegToRad(90.0), 0.0);
  EXPECT_NEAR(d.x, 0.0, 1e-12);
  EXPECT_NEAR(d.y, 1.0, 1e-12);
  EXPECT_NEAR(BeamModel::Direction(0.3, 0.2).Length(), 1.0, 1e-12);
}

TEST(Types, AnglesAndPoses)
{
  EXPECT_NEAR(WrapAngle(3 * kPi), kPi, 1e-12);
  EXPECT_NEAR(WrapAngle(-kPi), kPi, 1e-12);
  EXPECT_NEAR(WrapAngle(0.5), 0.5, 1e-12);
  EXPECT_NEAR(WrapAngle2Pi(-0.5), kTwoPi - 0.5, 1e-12);

  Pose p;
  p.position = {10, 20, 3};
  p.orientation = Quat::FromYaw(kPi / 2);
  const Vec3 w = p.Transform({1, 0, 0});
  EXPECT_NEAR(w.x, 10, 1e-12);
  EXPECT_NEAR(w.y, 21, 1e-12);
  const Vec3 back = p.InverseTransform(w);
  EXPECT_NEAR(back.x, 1, 1e-12);
  EXPECT_NEAR(back.y, 0, 1e-12);
  EXPECT_NEAR(p.Yaw(), kPi / 2, 1e-12);
  const Quat q = Quat::FromRPY(0.1, -0.2, 0.3);
  EXPECT_NEAR(q.Yaw(), 0.3, 1e-12);
}
