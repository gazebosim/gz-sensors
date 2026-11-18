// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Unit tests: configuration parsing, validation and derived values.

#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "marine_radar/RadarConfig.hh"

#include "test_config.hh"  // NOLINT(build/include)

using marine_radar::DegToRad;
using marine_radar::ParseConfig;
using marine_radar::RadarConfig;

namespace
{
bool AnyContains(const std::vector<std::string> &_v, const std::string &_s)
{
  for (const auto &x : _v)
    if (x.find(_s) != std::string::npos)
      return true;
  return false;
}
}  // namespace

TEST(RadarConfig, DefaultsAreValid)
{
  RadarConfig c;
  EXPECT_TRUE(c.Validate().empty());
  EXPECT_TRUE(c.FullCircle());
  EXPECT_EQ(c.NumSpokes(), 4096);
  EXPECT_EQ(c.NumBins(), 495);
  EXPECT_DOUBLE_EQ(c.ScanPeriod(), 2.5);
  EXPECT_FALSE(marine_radar::Summary(c).empty());
}

TEST(RadarConfig, EmptyMapGivesDefaults)
{
  const auto r = ParseConfig({});
  EXPECT_TRUE(r.Ok());
  EXPECT_TRUE(r.warnings.empty());
  EXPECT_DOUBLE_EQ(r.config.range_max, 1000.0);
}

TEST(RadarConfig, ParsesAllNumericParametersWithUnits)
{
  const std::map<std::string, std::string> p = {
    {"range_min", "20"}, {"range_max", " 500 "}, {"range_resolution", "5"},
    {"horizontal_fov_deg", "180"}, {"angular_resolution_deg", "0.5"},
    {"rotation_rate_rpm", "48"}, {"beamwidth_horizontal_deg", "2"},
    {"beamwidth_vertical_deg", "25"}, {"slice_heights", "1, 2 4"},
    {"snr_reference_db", "40"}, {"reference_range", "500"},
    {"detection_threshold_db", "10"}, {"intensity_noise_stddev_db", "0"},
    {"range_noise_stddev", "1.5"}, {"bearing_noise_stddev_deg", "0.2"},
    {"echo_detection_probability", "0.8"}, {"false_alarm_rate", "3"},
    {"sea_level_filter", "false"}, {"sea_level", "-1"},
    {"min_target_cells", "2"}, {"cluster_spoke_gap", "0"},
    {"cluster_bin_gap", "1"}, {"association_gate", "30"},
    {"track_coast_scans", "1"}, {"image_size", "256"}, {"image_rate", "2"},
    {"display_range", "400"}, {"range_rings", "5"},
    {"bearing_line_spacing_deg", "45"}, {"display_mode", "north_up"},
    {"afterglow", "0.2"}, {"draw_target_markers", "false"},
    {"echo_expansion", "0"}, {"seed", "7"}};
  const auto r = ParseConfig(p);
  ASSERT_TRUE(r.Ok()) << (r.errors.empty() ? "" : r.errors.front());
  EXPECT_TRUE(r.warnings.empty());
  const RadarConfig &c = r.config;
  EXPECT_DOUBLE_EQ(c.range_min, 20.0);
  EXPECT_DOUBLE_EQ(c.range_max, 500.0);
  EXPECT_DOUBLE_EQ(c.range_resolution, 5.0);
  EXPECT_NEAR(c.horizontal_fov, DegToRad(180.0), 1e-12);
  EXPECT_NEAR(c.angular_resolution, DegToRad(0.5), 1e-12);
  EXPECT_EQ(c.NumSpokes(), 360);
  EXPECT_EQ(c.NumBins(), 96);
  EXPECT_DOUBLE_EQ(c.rotation_rate_rpm, 48.0);
  EXPECT_NEAR(c.beamwidth_horizontal, DegToRad(2.0), 1e-12);
  EXPECT_NEAR(c.beamwidth_vertical, DegToRad(25.0), 1e-12);
  ASSERT_EQ(c.slice_heights.size(), 3u);
  EXPECT_DOUBLE_EQ(c.slice_heights[2], 4.0);
  EXPECT_DOUBLE_EQ(c.snr_reference_db, 40.0);
  EXPECT_DOUBLE_EQ(c.reference_range, 500.0);
  EXPECT_DOUBLE_EQ(c.detection_threshold_db, 10.0);
  EXPECT_DOUBLE_EQ(c.intensity_noise_stddev_db, 0.0);
  EXPECT_DOUBLE_EQ(c.range_noise_stddev, 1.5);
  EXPECT_NEAR(c.bearing_noise_stddev, DegToRad(0.2), 1e-12);
  EXPECT_DOUBLE_EQ(c.echo_detection_probability, 0.8);
  EXPECT_DOUBLE_EQ(c.false_alarm_rate, 3.0);
  EXPECT_FALSE(c.sea_level_filter);
  EXPECT_DOUBLE_EQ(c.sea_level, -1.0);
  EXPECT_EQ(c.min_target_cells, 2);
  EXPECT_EQ(c.cluster_spoke_gap, 0);
  EXPECT_EQ(c.cluster_bin_gap, 1);
  EXPECT_DOUBLE_EQ(c.association_gate, 30.0);
  EXPECT_EQ(c.track_coast_scans, 1);
  EXPECT_EQ(c.image_size, 256);
  EXPECT_DOUBLE_EQ(c.image_rate, 2.0);
  EXPECT_DOUBLE_EQ(c.EffectiveDisplayRange(), 400.0);
  EXPECT_EQ(c.range_rings, 5);
  EXPECT_NEAR(c.bearing_line_spacing, DegToRad(45.0), 1e-12);
  EXPECT_TRUE(c.north_up);
  EXPECT_DOUBLE_EQ(c.afterglow, 0.2);
  EXPECT_FALSE(c.draw_target_markers);
  EXPECT_FALSE(c.echo_expansion);
  EXPECT_EQ(c.seed, 7u);
}

TEST(RadarConfig, UnknownKeyIsWarning)
{
  const auto r = ParseConfig({{"max_range", "500"}});
  EXPECT_TRUE(r.Ok());
  ASSERT_EQ(r.warnings.size(), 1u);
  EXPECT_TRUE(AnyContains(r.warnings, "max_range"));
}

TEST(RadarConfig, MalformedValuesAreErrors)
{
  EXPECT_TRUE(AnyContains(ParseConfig({{"range_max", "1km"}}).errors,
                          "range_max"));
  EXPECT_TRUE(AnyContains(ParseConfig({{"range_max", ""}}).errors,
                          "range_max"));
  EXPECT_TRUE(AnyContains(ParseConfig({{"image_size", "12.5"}}).errors,
                          "image_size"));
  EXPECT_TRUE(AnyContains(ParseConfig({{"sea_level_filter", "maybe"}}).errors,
                          "sea_level_filter"));
  EXPECT_TRUE(AnyContains(ParseConfig({{"display_mode", "course_up"}}).errors,
                          "display_mode"));
  EXPECT_TRUE(AnyContains(ParseConfig({{"slice_heights", "1 two 3"}}).errors,
                          "slice_heights"));
  EXPECT_TRUE(AnyContains(ParseConfig({{"seed", "-1"}}).errors, "seed"));
  EXPECT_TRUE(AnyContains(ParseConfig({{"range_max", "nan"}}).errors,
                          "range_max"));
}

TEST(RadarConfig, RangeRules)
{
  auto r = ParseConfig({{"range_max", "5"}});
  ASSERT_FALSE(r.Ok());
  EXPECT_TRUE(AnyContains(r.errors, "<range_max>"));
  EXPECT_TRUE(AnyContains(r.errors, "greater than range_min"));

  r = ParseConfig({{"range_min", "-1"}});
  EXPECT_TRUE(AnyContains(r.errors, "<range_min>"));

  r = ParseConfig({{"range_resolution", "0"}});
  EXPECT_TRUE(AnyContains(r.errors, "<range_resolution>"));

  // Too many bins for a long range at fine resolution.
  r = ParseConfig({{"range_max", "100000"}, {"range_resolution", "1"}});
  EXPECT_TRUE(AnyContains(r.errors, "range bins"));

  // The three ranges from the requirements are valid.
  for (const char *rm : {"100", "500", "1000"})
    EXPECT_TRUE(ParseConfig({{"range_max", rm}}).Ok()) << rm;
}

TEST(RadarConfig, AngularRules)
{
  EXPECT_TRUE(AnyContains(
      ParseConfig({{"horizontal_fov_deg", "400"}}).errors,
      "horizontal_fov_deg"));
  EXPECT_TRUE(AnyContains(
      ParseConfig({{"horizontal_fov_deg", "0"}}).errors,
      "horizontal_fov_deg"));
  EXPECT_TRUE(AnyContains(
      ParseConfig({{"angular_resolution_deg", "-0.1"}}).errors,
      "angular_resolution_deg"));
  EXPECT_TRUE(AnyContains(
      ParseConfig({{"angular_resolution_deg", "0.001"}}).errors,
      "spokes per scan"));
  EXPECT_TRUE(AnyContains(
      ParseConfig({{"rotation_rate_rpm", "0"}}).errors, "rotation_rate_rpm"));
  EXPECT_TRUE(AnyContains(
      ParseConfig({{"beamwidth_horizontal_deg", "90"}}).errors,
      "beamwidth_horizontal_deg"));
}

TEST(RadarConfig, SamplesAndResolution)
{
  auto r = ParseConfig({{"samples", "2048"}});
  ASSERT_TRUE(r.Ok());
  EXPECT_EQ(r.config.NumSpokes(), 2048);
  EXPECT_NEAR(r.config.angular_resolution, 2 * marine_radar::kPi / 2048,
              1e-12);

  r = ParseConfig({{"samples", "360"}, {"angular_resolution_deg", "1"}});
  EXPECT_TRUE(r.Ok());

  r = ParseConfig({{"samples", "360"}, {"angular_resolution_deg", "0.5"}});
  EXPECT_TRUE(AnyContains(r.errors, "disagree"));

  r = ParseConfig({{"samples", "0"}});
  EXPECT_TRUE(AnyContains(r.errors, "samples"));

  // 0.7 deg does not divide 360: adjusted with a warning.
  r = ParseConfig({{"angular_resolution_deg", "0.7"}});
  ASSERT_TRUE(r.Ok());
  EXPECT_TRUE(AnyContains(r.warnings, "adjusted"));
  EXPECT_EQ(r.config.NumSpokes(), 514);
  EXPECT_NEAR(r.config.angular_resolution * r.config.NumSpokes(),
              2 * marine_radar::kPi, 1e-9);
}

TEST(RadarConfig, SliceAndDisplayRules)
{
  EXPECT_TRUE(AnyContains(ParseConfig({{"slice_heights", "-1"}}).errors,
                          "slice_heights"));
  EXPECT_TRUE(AnyContains(ParseConfig({{"image_size", "32"}}).errors,
                          "image_size"));
  EXPECT_TRUE(AnyContains(ParseConfig({{"afterglow", "1.5"}}).errors,
                          "afterglow"));
  EXPECT_TRUE(AnyContains(
      ParseConfig({{"echo_detection_probability", "1.1"}}).errors,
      "echo_detection_probability"));
  EXPECT_TRUE(AnyContains(ParseConfig({{"false_alarm_rate", "-2"}}).errors,
                          "false_alarm_rate"));
  EXPECT_TRUE(AnyContains(ParseConfig({{"min_target_cells", "0"}}).errors,
                          "min_target_cells"));
}

TEST(RadarConfig, ShippedConfigExamplesAreValid)
{
  // Every preset shipped in examples/marine_radar must parse without errors
  // or warnings. Keys consumed by MarineRadarSensor itself (rather than by
  // ParseConfig) are skipped.
  const std::vector<std::string> sensorKeys = {
    "topic", "update_rate", "always_on", "visualize", "frame_id",
    "publish_image", "publish_returns", "normal_estimation",
    "normal_probe_angle_deg"};
  for (const std::string name : {"radar_short_range_100m.sdf",
                                 "radar_medium_range_500m.sdf",
                                 "radar_long_range_1000m.sdf"})
  {
    std::ifstream f(std::string(PROJECT_SOURCE_PATH) +
        "/examples/marine_radar/" + name);
    if (!f.good())
    {
      // The source tree is not reachable (e.g. a sandboxed build without the
      // examples directory staged as test data).
      GTEST_SKIP() << "Cannot read examples/marine_radar/" << name;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string text = ss.str();
    // Drop XML comments, then collect <key>value</key> pairs.
    text = std::regex_replace(text, std::regex("<!--[\\s\\S]*?-->"), "");
    std::map<std::string, std::string> params;
    const std::regex kv("<([a-z_]+)>([^<]*)</\\1>");
    for (auto it = std::sregex_iterator(text.begin(), text.end(), kv);
         it != std::sregex_iterator(); ++it)
    {
      const std::string key = (*it)[1];
      if (std::find(sensorKeys.begin(), sensorKeys.end(), key) ==
          sensorKeys.end())
        params[key] = (*it)[2];
    }
    ASSERT_GE(params.size(), 5u) << name;
    const auto r = ParseConfig(params);
    EXPECT_TRUE(r.Ok()) << name << ": "
                        << (r.errors.empty() ? "" : r.errors.front());
    EXPECT_TRUE(r.warnings.empty()) << name << ": "
                                    << (r.warnings.empty() ? "" :
                                        r.warnings.front());
  }
}

TEST(RadarConfig, DerivedGeometry)
{
  RadarConfig c;
  c.horizontal_fov = DegToRad(90.0);
  c.angular_resolution = DegToRad(1.0);
  ASSERT_TRUE(c.Validate().empty());
  EXPECT_FALSE(c.FullCircle());
  EXPECT_EQ(c.NumSpokes(), 90);
  // Spoke azimuths are centred in the FOV, symmetric about +X.
  EXPECT_NEAR(c.SpokeAzimuth(0), DegToRad(-44.5), 1e-12);
  EXPECT_NEAR(c.SpokeAzimuth(89), DegToRad(44.5), 1e-12);
  EXPECT_NEAR(c.BinCenterRange(0), c.range_min + c.range_resolution / 2,
              1e-12);
  EXPECT_NEAR(c.SpokeDwell() * 360, c.ScanPeriod(), 1e-9);
}
