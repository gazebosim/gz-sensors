// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Unit tests: echo formation (radar equation, threshold, noise, missed and
// false echoes, sea-level filter).

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <utility>
#include <vector>

#include "marine_radar/EchoModel.hh"
#include "marine_radar/ScanGeometry.hh"

using namespace marine_radar;  // NOLINT

namespace
{
RadarConfig QuietConfig()
{
  RadarConfig c;
  c.intensity_noise_stddev_db = 0.0;
  c.range_noise_stddev = 0.0;
  return c;
}

RawSpoke OneHit(double _range, double _cos = 1.0, double _z = 1.0,
                double _el = 0.0)
{
  RawSpoke s;
  s.hits.push_back({_range, _el, _cos, _z});
  return s;
}
}  // namespace

TEST(EchoModel, SingleHitBinAndSnr)
{
  const RadarConfig c = QuietConfig();
  BeamModel beam(c);
  EchoModel m(c, beam);
  std::mt19937 rng(1);
  const RawSpoke s = OneHit(500.0);
  const auto echoes = m.Form({{0.0, &s}}, rng);
  ASSERT_EQ(echoes.size(), 1u);
  EXPECT_EQ(echoes[0].bin, static_cast<int>((500.0 - 10.0) / 2.0));
  // 30 dB at 1000 m -> +12.04 dB at 500 m.
  EXPECT_NEAR(echoes[0].snr_db, 30.0 + 40.0 * std::log10(2.0), 1e-4);
  EXPECT_FLOAT_EQ(echoes[0].range, 500.0f);
  EXPECT_FALSE(echoes[0].false_alarm);
  EXPECT_NEAR(m.HitSnrDb(s.hits[0], 0.0), echoes[0].snr_db, 1e-4);
}

TEST(EchoModel, RangeGates)
{
  const RadarConfig c = QuietConfig();
  BeamModel beam(c);
  EchoModel m(c, beam);
  std::mt19937 rng(1);
  const RawSpoke tooClose = OneHit(5.0);
  const RawSpoke tooFar = OneHit(1000.0);
  const RawSpoke edge = OneHit(999.9);
  EXPECT_TRUE(m.Form({{0.0, &tooClose}}, rng).empty());
  EXPECT_TRUE(m.Form({{0.0, &tooFar}}, rng).empty());
  const auto e = m.Form({{0.0, &edge}}, rng);
  ASSERT_EQ(e.size(), 1u);
  EXPECT_EQ(e[0].bin, c.NumBins() - 1);
}

TEST(EchoModel, RadarEquationTrends)
{
  const RadarConfig c = QuietConfig();
  BeamModel beam(c);
  EchoModel m(c, beam);
  RawHit h{200.0, 0.0, 1.0, 1.0};
  const double base = m.HitSnrDb(h, 0.0);
  RawHit far = h;
  far.range = 400.0;
  EXPECT_NEAR(base - m.HitSnrDb(far, 0.0), 12.0412, 1e-3);   // R^-4
  RawHit oblique = h;
  oblique.cos_incidence = 0.1;
  EXPECT_NEAR(base - m.HitSnrDb(oblique, 0.0), 10.0, 1e-9);  // |cos|
  EXPECT_NEAR(base - m.HitSnrDb(h, c.beamwidth_horizontal / 2), 3.0103,
              1e-3);                                         // beam edge
  RawHit low = h;
  low.elevation = -c.beamwidth_vertical / 2;
  EXPECT_NEAR(base - m.HitSnrDb(low, 0.0), 3.0103, 1e-3);
}

TEST(EchoModel, ThresholdDropsWeakEchoes)
{
  RadarConfig c = QuietConfig();
  c.snr_reference_db = 0.0;  // 0 dB at 1000 m
  c.range_max = 5000.0;
  c.range_resolution = 5.0;
  BeamModel beam(c);
  EchoModel m(c, beam);
  std::mt19937 rng(1);
  // At 500 m: +12 dB > 8 dB threshold. At 900 m: +1.8 dB < 8 dB.
  const RawSpoke near = OneHit(500.0);
  const RawSpoke far = OneHit(900.0);
  EXPECT_EQ(m.Form({{0.0, &near}}, rng).size(), 1u);
  EXPECT_TRUE(m.Form({{0.0, &far}}, rng).empty());
}

TEST(EchoModel, SameBinReturnsAddInPower)
{
  const RadarConfig c = QuietConfig();
  BeamModel beam(c);
  EchoModel m(c, beam);
  std::mt19937 rng(1);
  RawSpoke s;
  s.hits.push_back({300.1, 0.0, 1.0, 1.0});
  s.hits.push_back({300.2, 0.0, 1.0, 3.0});
  const auto e = m.Form({{0.0, &s}}, rng);
  ASSERT_EQ(e.size(), 1u);
  const double single = m.HitSnrDb(s.hits[0], 0.0);
  EXPECT_NEAR(e[0].snr_db, single + 3.0103, 0.01);
}

TEST(EchoModel, BeamSmearingFromNeighbours)
{
  const RadarConfig c = QuietConfig();
  BeamModel beam(c);
  EchoModel m(c, beam);
  std::mt19937 rng(1);
  const RawSpoke s = OneHit(300.0);
  // Seen from a spoke one beam width away, the echo is 12 dB weaker.
  const auto onAxis = m.Form({{0.0, &s}}, rng);
  const auto offAxis = m.Form({{c.beamwidth_horizontal, &s}}, rng);
  ASSERT_EQ(onAxis.size(), 1u);
  ASSERT_EQ(offAxis.size(), 1u);
  EXPECT_NEAR(onAxis[0].snr_db - offAxis[0].snr_db, 12.04, 0.01);
}

TEST(EchoModel, SeaLevelFilter)
{
  RadarConfig c = QuietConfig();
  BeamModel beam(c);
  std::mt19937 rng(1);
  const RawSpoke underwater = OneHit(300.0, 1.0, -0.5);
  EXPECT_TRUE(EchoModel(c, beam).Form({{0.0, &underwater}}, rng).empty());
  c.sea_level_filter = false;
  EXPECT_EQ(EchoModel(c, beam).Form({{0.0, &underwater}}, rng).size(), 1u);
  c.sea_level_filter = true;
  c.sea_level = -2.0;
  EXPECT_EQ(EchoModel(c, beam).Form({{0.0, &underwater}}, rng).size(), 1u);
}

TEST(EchoModel, RangeNoiseStatistics)
{
  RadarConfig c = QuietConfig();
  c.range_noise_stddev = 3.0;
  BeamModel beam(c);
  EchoModel m(c, beam);
  std::mt19937 rng(11);
  const RawSpoke s = OneHit(500.0);
  double sum = 0.0, sum2 = 0.0;
  int n = 0;
  for (int i = 0; i < 5000; ++i)
  {
    const auto e = m.Form({{0.0, &s}}, rng);
    ASSERT_EQ(e.size(), 1u);
    sum += e[0].range;
    sum2 += e[0].range * e[0].range;
    ++n;
  }
  const double mean = sum / n;
  const double sd = std::sqrt(sum2 / n - mean * mean);
  EXPECT_NEAR(mean, 500.0, 0.2);
  EXPECT_NEAR(sd, 3.0, 0.15);
}

TEST(EchoModel, MissedEchoesFollowProbability)
{
  RadarConfig c = QuietConfig();
  BeamModel beam(c);
  std::mt19937 rng(5);
  const RawSpoke s = OneHit(300.0);

  c.echo_detection_probability = 0.0;
  EXPECT_TRUE(EchoModel(c, beam).Form({{0.0, &s}}, rng).empty());

  c.echo_detection_probability = 0.3;
  EchoModel m(c, beam);
  int kept = 0;
  const int trials = 20000;
  for (int i = 0; i < trials; ++i)
    kept += m.Form({{0.0, &s}}, rng).empty() ? 0 : 1;
  EXPECT_NEAR(static_cast<double>(kept) / trials, 0.3, 0.015);
}

TEST(EchoModel, FalseAlarmRatePerScan)
{
  RadarConfig c = QuietConfig();
  c.false_alarm_rate = 5.0;  // per scan of NumSpokes() spokes
  BeamModel beam(c);
  EchoModel m(c, beam);
  std::mt19937 rng(9);
  const int scans = 200;
  int falseEchoes = 0;
  for (int i = 0; i < scans * c.NumSpokes(); ++i)
  {
    for (const auto &e : m.Form({}, rng))
    {
      EXPECT_TRUE(e.false_alarm);
      EXPECT_GE(e.snr_db, c.detection_threshold_db);
      EXPECT_GE(e.bin, 0);
      EXPECT_LT(e.bin, c.NumBins());
      ++falseEchoes;
    }
  }
  EXPECT_NEAR(static_cast<double>(falseEchoes) / scans, 5.0, 0.5);
}

TEST(EchoModel, DisplayLevels)
{
  const RadarConfig c = QuietConfig();
  BeamModel beam(c);
  EchoModel m(c, beam);
  EXPECT_EQ(m.DisplayLevel(c.detection_threshold_db - 0.1), 0);
  EXPECT_EQ(m.DisplayLevel(c.detection_threshold_db), 1);
  EXPECT_EQ(m.DisplayLevel(c.detection_threshold_db +
                           c.display_dynamic_range_db), 255);
  EXPECT_EQ(m.DisplayLevel(200.0), 255);
  EXPECT_LT(m.DisplayLevel(15.0), m.DisplayLevel(25.0));
}
