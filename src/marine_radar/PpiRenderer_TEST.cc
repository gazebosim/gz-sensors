// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Unit tests: PPI radar image generation.

#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "marine_radar/PpiRenderer.hh"

using namespace marine_radar;  // NOLINT

namespace
{
RadarConfig ImgConfig()
{
  RadarConfig c;
  c.image_size = 200;
  c.afterglow = 0.0;
  c.echo_expansion = false;
  return c;
}

int Slots(const RadarConfig &_c)
{
  return static_cast<int>(std::lround(kTwoPi / _c.SpokeSpacing()));
}

bool IsEchoColour(const RgbImage &_img, int _u, int _v)
{
  const uint8_t *p = _img.Pixel(_u, _v);
  return p[1] >= 105 && p[2] <= 75;
}

int CountEcho(const RgbImage &_img)
{
  int n = 0;
  for (int v = 0; v < _img.height; ++v)
    for (int u = 0; u < _img.width; ++u)
      n += IsEchoColour(_img, u, v) ? 1 : 0;
  return n;
}

bool AnyNear(const RgbImage &_img, double _u, double _v, int _w,
             bool (*_pred)(const uint8_t *))
{
  for (int dv = -_w; dv <= _w; ++dv)
  {
    for (int du = -_w; du <= _w; ++du)
    {
      const int u = static_cast<int>(std::lround(_u)) + du;
      const int v = static_cast<int>(std::lround(_v)) + dv;
      if (u >= 0 && v >= 0 && u < _img.width && v < _img.height &&
          _pred(_img.Pixel(u, v)))
        return true;
    }
  }
  return false;
}

bool Cyan(const uint8_t *_p)
{
  return _p[0] == 0 && _p[1] == 220 && _p[2] == 255;
}
bool EchoPx(const uint8_t *_p) { return _p[1] >= 105 && _p[2] <= 75; }
}  // namespace

TEST(PpiRenderer, BackgroundCentreAndSize)
{
  const RadarConfig c = ImgConfig();
  DisplayBuffer buf(Slots(c), c.NumBins());
  PpiRenderer r(c);
  RgbImage img;
  r.Render(buf, 1.0, 1.0, Pose(), {}, img);
  ASSERT_EQ(img.width, 200);
  ASSERT_EQ(img.height, 200);
  ASSERT_EQ(img.data.size(), 200u * 200u * 3u);
  const uint8_t *corner = img.Pixel(0, 0);
  EXPECT_EQ(corner[0], 6);
  EXPECT_EQ(corner[1], 8);
  EXPECT_EQ(corner[2], 10);
  const uint8_t *centre = img.Pixel(100, 100);
  EXPECT_EQ(centre[0], 255);
  EXPECT_EQ(centre[1], 255);
  EXPECT_EQ(centre[2], 255);
  EXPECT_EQ(CountEcho(img), 0);
}

TEST(PpiRenderer, EchoAppearsAtRangeAndBearing)
{
  const RadarConfig c = ImgConfig();
  DisplayBuffer buf(Slots(c), c.NumBins());
  PpiRenderer r(c);
  const int bin = static_cast<int>((500.0 - c.range_min) / c.range_resolution);
  // Port side (radar azimuth +90 deg) is on the left of a head-up display.
  buf.Paint(r.DisplayAngle(DegToRad(90.0), 0.0), 1.0, {{bin, 200}});
  RgbImage img;
  r.Render(buf, 1.0, 0.3, Pose(), {}, img);
  double u, v;
  r.ToPixel(c.BinCenterRange(bin), DegToRad(90.0), u, v);
  EXPECT_LT(u, 60.0);
  EXPECT_NEAR(v, 99.5, 1e-9);
  EXPECT_TRUE(AnyNear(img, u, v, 1, EchoPx));
  EXPECT_GE(CountEcho(img), 1);
}

TEST(PpiRenderer, SubPixelEchoIsNotLost)
{
  RadarConfig c = ImgConfig();
  c.range_max = 5000.0;
  c.range_resolution = 2.0;  // 2 m bins, ~50 m per pixel
  DisplayBuffer buf(Slots(c), c.NumBins());
  PpiRenderer r(c);
  buf.Paint(0.3, 1.0, {{1000, 120}});
  RgbImage img;
  r.Render(buf, 1.0, 2.0, Pose(), {}, img);
  EXPECT_GE(CountEcho(img), 1);
}

TEST(PpiRenderer, EchoExpansionGrowsBlips)
{
  RadarConfig c = ImgConfig();
  DisplayBuffer buf(Slots(c), c.NumBins());
  buf.Paint(0.7, 1.0, {{300, 150}});
  RgbImage a, b;
  PpiRenderer(c).Render(buf, 1.0, 2.0, Pose(), {}, a);
  c.echo_expansion = true;
  PpiRenderer(c).Render(buf, 1.0, 2.0, Pose(), {}, b);
  EXPECT_GT(CountEcho(b), CountEcho(a));
}

TEST(PpiRenderer, AfterglowFadesAndExpires)
{
  RadarConfig c = ImgConfig();
  c.afterglow = 0.6;
  DisplayBuffer buf(Slots(c), c.NumBins());
  PpiRenderer r(c);
  buf.Paint(0.0, 10.0, {{200, 255}});
  double u, v;
  r.ToPixel(c.BinCenterRange(200), 0.0, u, v);
  auto green = [&](double _t)
  {
    RgbImage img;
    r.Render(buf, _t, 2.0, Pose(), {}, img);
    int g = 0;
    for (int dv = -2; dv <= 2; ++dv)
      for (int du = -2; du <= 2; ++du)
      {
        const uint8_t *p = img.Pixel(static_cast<int>(std::lround(u)) + du,
                                     static_cast<int>(std::lround(v)) + dv);
        if (p[2] <= 75 && p[0] > 100)  // orange/yellow echo (level 255)
          g = std::max(g, static_cast<int>(p[0]));
      }
    return g;
  };
  const int fresh = green(10.0);
  const int halfScan = green(10.0 + c.ScanPeriod() / 2);
  const int expired = green(10.0 + 2.0 * c.ScanPeriod());
  EXPECT_GT(fresh, halfScan);
  EXPECT_NEAR(halfScan, fresh * 0.7, 3.0);
  EXPECT_EQ(expired, 0);
}

TEST(PpiRenderer, RangeRingsPresent)
{
  const RadarConfig c = ImgConfig();
  DisplayBuffer buf(Slots(c), c.NumBins());
  RgbImage img;
  PpiRenderer(c).Render(buf, 1.0, 2.0, Pose(), {}, img);
  // Walk up column 99 (left of the heading line) and count ring crossings.
  int rings = 0;
  bool inRing = false;
  for (int v = 99; v >= 0; --v)
  {
    const uint8_t *p = img.Pixel(99, v);
    const bool ring = p[0] == 40 && p[1] == 100 && p[2] == 64;
    if (ring && !inRing)
      ++rings;
    inRing = ring;
  }
  EXPECT_EQ(rings, c.range_rings);
}

TEST(PpiRenderer, NorthUpDisplayAngles)
{
  RadarConfig c = ImgConfig();
  c.north_up = true;
  PpiRenderer r(c);
  // Heading north (yaw 90 deg): the bow is up and port is to the left.
  EXPECT_NEAR(r.DisplayAngle(0.0, kPi / 2), 0.0, 1e-12);
  EXPECT_NEAR(r.DisplayAngle(kPi / 2, kPi / 2), kPi / 2, 1e-12);
  // Heading east (yaw 0): the bow points right (display angle -90 deg).
  EXPECT_NEAR(r.DisplayAngle(0.0, 0.0), -kPi / 2, 1e-12);
  c.north_up = false;
  PpiRenderer h(c);
  EXPECT_NEAR(h.DisplayAngle(0.4, 1.0), 0.4, 1e-12);
}

TEST(PpiRenderer, TargetMarkersFollowCurrentPose)
{
  const RadarConfig c = ImgConfig();
  DisplayBuffer buf(Slots(c), c.NumBins());
  PpiRenderer r(c);
  Target t;
  t.position_world = {0.0, 500.0, 0.0};  // north of the origin
  t.range = 500.0;
  t.range_extent = 4.0;
  t.bearing_extent = DegToRad(2.0);
  RgbImage img;
  // Vessel at the origin heading east: target is on the port beam (left).
  r.Render(buf, 1.0, 2.0, Pose(), {t}, img);
  double u, v;
  r.ToPixel(500.0, kPi / 2, u, v);
  EXPECT_TRUE(AnyNear(img, u, v, 6, Cyan));
  // Vessel heading north: the target is dead ahead (up).
  Pose north;
  north.orientation = Quat::FromYaw(kPi / 2);
  r.Render(buf, 1.0, 2.0, north, {t}, img);
  r.ToPixel(500.0, 0.0, u, v);
  EXPECT_TRUE(AnyNear(img, u, v, 6, Cyan));
  // Targets beyond the display range are not drawn.
  t.position_world = {0.0, 5000.0, 0.0};
  r.Render(buf, 1.0, 2.0, Pose(), {t}, img);
  int cyan = 0;
  for (int vv = 0; vv < img.height; ++vv)
    for (int uu = 0; uu < img.width; ++uu)
      cyan += Cyan(img.Pixel(uu, vv)) ? 1 : 0;
  EXPECT_EQ(cyan, 0);
}

TEST(DisplayBuffer, PaintOverwritesSlot)
{
  DisplayBuffer buf(360, 100);
  buf.Paint(0.0, 1.0, {{10, 50}, {20, 60}, {10, 70}});
  const int s = buf.SlotForAngle(0.0);
  EXPECT_EQ(buf.Level(s, 10), 70);
  EXPECT_EQ(buf.Level(s, 20), 60);
  EXPECT_EQ(buf.SlotEchoes(s).size(), 2u);
  buf.Paint(0.0, 2.0, {{30, 40}});
  EXPECT_EQ(buf.Level(s, 10), 0);
  EXPECT_EQ(buf.Level(s, 30), 40);
  EXPECT_DOUBLE_EQ(buf.Time(s), 2.0);
  EXPECT_EQ(buf.SlotEchoes(s).size(), 1u);
  buf.Clear();
  EXPECT_EQ(buf.Level(s, 30), 0);
  EXPECT_TRUE(buf.SlotEchoes(s).empty());
  EXPECT_EQ(buf.SlotForAngle(kTwoPi - 1e-9), 359);
  EXPECT_EQ(buf.SlotForAngle(-1e-9), 359);
}
