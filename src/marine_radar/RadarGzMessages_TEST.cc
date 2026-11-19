// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Unit tests: Gazebo message generation (point cloud layouts, image, header).

#include <gtest/gtest.h>

#include <gz/msgs/PointCloudPackedUtils.hh>

#include <cstring>
#include <string>

#include "marine_radar/RadarGzMessages.hh"

using marine_radar::RgbImage;
using marine_radar::ScanResult;
using marine_radar::Target;

namespace
{
template <typename T>
T Get(const gz::msgs::PointCloudPacked &_m, size_t _i, const std::string &_f)
{
  for (const auto &f : _m.field())
  {
    if (f.name() == _f)
    {
      T v;
      std::memcpy(&v, _m.data().data() + _i * _m.point_step() + f.offset(),
                  sizeof(T));
      return v;
    }
  }
  ADD_FAILURE() << "missing field " << _f;
  return T();
}

std::string Frame(const gz::msgs::Header &_h)
{
  for (const auto &d : _h.data())
    if (d.key() == "frame_id")
      return d.value(0);
  return "";
}
}  // namespace

TEST(GzMessages, HeaderStamp)
{
  gz::msgs::Header h;
  marine_radar::FillHeader(12.25, "radar_link", h);
  EXPECT_EQ(h.stamp().sec(), 12);
  EXPECT_EQ(h.stamp().nsec(), 250000000);
  EXPECT_EQ(Frame(h), "radar_link");
  marine_radar::FillHeader(3.9999999999, "f", h);
  EXPECT_EQ(h.stamp().sec(), 4);
  EXPECT_EQ(h.stamp().nsec(), 0);
}

TEST(GzMessages, TargetsLayout)
{
  ScanResult scan;
  scan.stamp = 5.0;
  Target t;
  t.id = 7;
  t.age = 3;
  t.range = 123.5;
  t.bearing = -0.25;
  t.elevation = 0.01;
  t.position_radar = {1, 2, 3};
  t.position_world = {1000.125, -2000.25, 4.5};
  t.intensity_db = 33.0;
  t.min_range = 120.0;
  t.range_extent = 6.0;
  t.bearing_extent = 0.05;
  t.num_cells = 42;
  t.time = 4.0;
  scan.targets = {t, t};
  scan.targets[1].id = 8;

  gz::msgs::PointCloudPacked m;
  marine_radar::FillTargets(scan, "radar_link", m);
  EXPECT_EQ(m.width(), 2u);
  EXPECT_EQ(m.height(), 1u);
  EXPECT_EQ(m.point_step(), 80u);
  EXPECT_EQ(m.point_step() % 8, 0u);
  EXPECT_EQ(m.row_step(), 160u);
  EXPECT_EQ(m.data().size(), 160u);
  EXPECT_TRUE(m.is_dense());
  EXPECT_EQ(Frame(m.header()), "radar_link");
  EXPECT_EQ(m.header().stamp().sec(), 5);
  // Every field naturally aligned.
  for (const auto &f : m.field())
  {
    const uint32_t size =
        f.datatype() == gz::msgs::PointCloudPacked::Field::FLOAT64 ? 8 : 4;
    EXPECT_EQ(f.offset() % size, 0u) << f.name();
  }
  EXPECT_DOUBLE_EQ(Get<double>(m, 0, "world_x"), 1000.125);
  EXPECT_DOUBLE_EQ(Get<double>(m, 0, "world_y"), -2000.25);
  EXPECT_DOUBLE_EQ(Get<double>(m, 0, "world_z"), 4.5);
  EXPECT_FLOAT_EQ(Get<float>(m, 0, "x"), 1.0f);
  EXPECT_FLOAT_EQ(Get<float>(m, 0, "y"), 2.0f);
  EXPECT_FLOAT_EQ(Get<float>(m, 0, "z"), 3.0f);
  EXPECT_FLOAT_EQ(Get<float>(m, 0, "range"), 123.5f);
  EXPECT_FLOAT_EQ(Get<float>(m, 0, "bearing"), -0.25f);
  EXPECT_FLOAT_EQ(Get<float>(m, 0, "elevation"), 0.01f);
  EXPECT_FLOAT_EQ(Get<float>(m, 0, "intensity"), 33.0f);
  EXPECT_EQ(Get<uint32_t>(m, 0, "id"), 7u);
  EXPECT_EQ(Get<uint32_t>(m, 1, "id"), 8u);
  EXPECT_EQ(Get<uint32_t>(m, 0, "age"), 3u);
  EXPECT_EQ(Get<uint32_t>(m, 0, "num_cells"), 42u);
  EXPECT_FLOAT_EQ(Get<float>(m, 0, "min_range"), 120.0f);
  EXPECT_FLOAT_EQ(Get<float>(m, 0, "range_extent"), 6.0f);
  EXPECT_FLOAT_EQ(Get<float>(m, 0, "bearing_extent"), 0.05f);
  EXPECT_FLOAT_EQ(Get<float>(m, 0, "time_offset"), -1.0f);

  // An empty scan is still a valid (empty) message with all fields.
  marine_radar::FillTargets(ScanResult(), "radar_link", m);
  EXPECT_EQ(m.width(), 0u);
  EXPECT_EQ(m.field_size(), 17);
  EXPECT_TRUE(m.data().empty());
}

TEST(GzMessages, ReturnsLayout)
{
  ScanResult scan;
  scan.stamp = 2.0;
  marine_radar::ReturnPoint r;
  r.position_radar = {10, -5, 1};
  r.position_world = {7, 8, 9};
  r.range = 11.2;
  r.bearing = -0.46;
  r.intensity_db = 20;
  r.time = 1.5;
  scan.returns = {r};
  gz::msgs::PointCloudPacked m;
  marine_radar::FillReturns(scan, "radar", m);
  EXPECT_EQ(m.width(), 1u);
  EXPECT_EQ(m.point_step(), 56u);
  EXPECT_DOUBLE_EQ(Get<double>(m, 0, "world_y"), 8.0);
  EXPECT_FLOAT_EQ(Get<float>(m, 0, "x"), 10.0f);
  EXPECT_FLOAT_EQ(Get<float>(m, 0, "intensity"), 20.0f);
  EXPECT_FLOAT_EQ(Get<float>(m, 0, "time_offset"), -0.5f);
}

TEST(GzMessages, Image)
{
  RgbImage img;
  img.Resize(64, 32);
  img.Pixel(3, 2)[1] = 200;
  gz::msgs::Image m;
  marine_radar::FillImage(img, 1.5, "radar_link", m);
  EXPECT_EQ(m.width(), 64u);
  EXPECT_EQ(m.height(), 32u);
  EXPECT_EQ(m.step(), 192u);
  EXPECT_EQ(m.pixel_format_type(), gz::msgs::PixelFormatType::RGB_INT8);
  EXPECT_EQ(m.data().size(), 64u * 32u * 3u);
  EXPECT_EQ(static_cast<uint8_t>(m.data()[(2 * 64 + 3) * 3 + 1]), 200);
  EXPECT_EQ(Frame(m.header()), "radar_link");
}
