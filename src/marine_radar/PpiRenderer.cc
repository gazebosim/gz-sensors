// Copyright 2026 marine_radar contributors. Apache-2.0.

#include "marine_radar/PpiRenderer.hh"

#include <algorithm>
#include <cmath>

namespace marine_radar
{
namespace
{
const uint8_t kOutside[3] = {6, 8, 10};
const uint8_t kInside[3] = {8, 20, 14};
const uint8_t kRing[3] = {40, 100, 64};
const uint8_t kBearingLine[3] = {26, 68, 44};
const uint8_t kHeading[3] = {170, 225, 185};
const uint8_t kSweep[3] = {90, 255, 140};
const uint8_t kMarker[3] = {0, 220, 255};
const uint8_t kCenter[3] = {255, 255, 255};

void Lerp(const double _a[3], const double _b[3], double _t, uint8_t _out[3])
{
  for (int k = 0; k < 3; ++k)
  {
    _out[k] = static_cast<uint8_t>(
        std::lround(_a[k] + (_b[k] - _a[k]) * _t));
  }
}

void SetPixel(RgbImage &_img, int _u, int _v, const uint8_t _rgb[3])
{
  if (_u < 0 || _v < 0 || _u >= _img.width || _v >= _img.height)
    return;
  uint8_t *p = _img.Pixel(_u, _v);
  p[0] = _rgb[0];
  p[1] = _rgb[1];
  p[2] = _rgb[2];
}
}  // namespace

/////////////////////////////////////////////////
DisplayBuffer::DisplayBuffer(int _slots, int _bins)
  : slots_(std::max(1, _slots)), bins_(std::max(1, _bins)),
    levels_(static_cast<size_t>(slots_) * static_cast<size_t>(bins_), 0u),
    time_(static_cast<size_t>(slots_), -1.0e9),
    sparse_(static_cast<size_t>(slots_))
{
}

/////////////////////////////////////////////////
int DisplayBuffer::SlotForAngle(double _a) const
{
  const double w = WrapAngle2Pi(_a);
  int s = static_cast<int>(std::floor(w / kTwoPi * slots_));
  return std::min(std::max(s, 0), slots_ - 1);
}

/////////////////////////////////////////////////
void DisplayBuffer::Paint(double _angle, double _time,
                          const std::vector<std::pair<int, uint8_t>> &_levels)
{
  const int s = SlotForAngle(_angle);
  uint8_t *row = &levels_[static_cast<size_t>(s) * static_cast<size_t>(bins_)];
  auto &sparse = sparse_[static_cast<size_t>(s)];
  for (const auto &bl : sparse)
    row[bl.first] = 0u;
  sparse.clear();
  for (const auto &bl : _levels)
  {
    if (bl.first < 0 || bl.first >= bins_ || bl.second == 0)
      continue;
    if (row[bl.first] == 0)
      sparse.emplace_back(bl.first, bl.second);
    row[bl.first] = std::max(row[bl.first], bl.second);
  }
  for (auto &bl : sparse)
    bl.second = row[bl.first];
  time_[static_cast<size_t>(s)] = _time;
}

/////////////////////////////////////////////////
void DisplayBuffer::Clear()
{
  std::fill(levels_.begin(), levels_.end(), static_cast<uint8_t>(0));
  std::fill(time_.begin(), time_.end(), -1.0e9);
  for (auto &s : sparse_)
    s.clear();
}

/////////////////////////////////////////////////
PpiRenderer::PpiRenderer(const RadarConfig &_cfg)
  : cfg_(_cfg),
    size_(_cfg.image_size),
    center_((_cfg.image_size - 1) / 2.0),
    radiusPx_(_cfg.image_size / 2.0 - 2.0),
    mPerPx_(_cfg.EffectiveDisplayRange() / (_cfg.image_size / 2.0 - 2.0)),
    scanPeriod_(_cfg.ScanPeriod()),
    bins_(_cfg.NumBins())
{
  const int slots = static_cast<int>(std::lround(kTwoPi / _cfg.SpokeSpacing()));
  DisplayBuffer probe(slots, bins_);
  const double displayRange = _cfg.EffectiveDisplayRange();
  const size_t npx = static_cast<size_t>(size_) * static_cast<size_t>(size_);
  cellLut_.assign(npx, -1);
  overlay_.assign(npx, 0u);

  for (int v = 0; v < size_; ++v)
  {
    for (int u = 0; u < size_; ++u)
    {
      const size_t idx = static_cast<size_t>(v) * static_cast<size_t>(size_) +
                         static_cast<size_t>(u);
      const double dxPx = u - center_;
      const double dyPx = center_ - v;
      const double rPx = std::hypot(dxPx, dyPx);
      if (rPx > radiusPx_ + 0.5)
        continue;
      overlay_[idx] = 1;
      const double r = rPx * mPerPx_;
      const double theta = std::atan2(-dxPx, dyPx);

      if (r <= displayRange && r >= _cfg.range_min && r < _cfg.range_max)
      {
        int bin = static_cast<int>(
            std::floor((r - _cfg.range_min) / _cfg.range_resolution));
        bin = std::min(std::max(bin, 0), bins_ - 1);
        const int slot = probe.SlotForAngle(theta);
        cellLut_[idx] = slot * bins_ + bin;
      }

      bool ring = false;
      for (int k = 1; k <= _cfg.range_rings; ++k)
      {
        const double ringPx = radiusPx_ * k / _cfg.range_rings;
        if (std::abs(rPx - ringPx) <= 0.5)
          ring = true;
      }
      if (std::abs(rPx - radiusPx_) <= 0.5)
        ring = true;
      if (ring)
      {
        overlay_[idx] = 2;
        continue;
      }
      if (_cfg.bearing_line_spacing > 0.0 && rPx > 3.0)
      {
        const double sp = _cfg.bearing_line_spacing;
        const double line = std::round(theta / sp) * sp;
        if (rPx * std::abs(std::sin(theta - line)) <= 0.6 &&
            std::cos(theta - line) > 0.0)
        {
          overlay_[idx] = 3;
        }
      }
    }
  }

  // Forward-mapping tables (echo "splatting").
  slotSin_.resize(static_cast<size_t>(probe.Slots()));
  slotCos_.resize(static_cast<size_t>(probe.Slots()));
  for (int s = 0; s < probe.Slots(); ++s)
  {
    const double a = (s + 0.5) * kTwoPi / probe.Slots();
    slotSin_[static_cast<size_t>(s)] = std::sin(a);
    slotCos_[static_cast<size_t>(s)] = std::cos(a);
  }
  binPx_.resize(static_cast<size_t>(bins_));
  for (int b = 0; b < bins_; ++b)
    binPx_[static_cast<size_t>(b)] = _cfg.BinCenterRange(b) / mPerPx_;

  // Echo palette: dark green -> green -> yellow -> orange.
  palette_.assign(256 * 3, 0u);
  const double c0[3] = {25, 110, 40};
  const double c1[3] = {60, 225, 70};
  const double c2[3] = {245, 235, 60};
  const double c3[3] = {255, 135, 30};
  for (int l = 1; l < 256; ++l)
  {
    const double t = (l - 1) / 254.0;
    uint8_t rgb[3];
    if (t < 1.0 / 3.0)
      Lerp(c0, c1, t * 3.0, rgb);
    else if (t < 2.0 / 3.0)
      Lerp(c1, c2, (t - 1.0 / 3.0) * 3.0, rgb);
    else
      Lerp(c2, c3, (t - 2.0 / 3.0) * 3.0, rgb);
    palette_[static_cast<size_t>(l) * 3 + 0] = rgb[0];
    palette_[static_cast<size_t>(l) * 3 + 1] = rgb[1];
    palette_[static_cast<size_t>(l) * 3 + 2] = rgb[2];
  }
}

/////////////////////////////////////////////////
double PpiRenderer::DisplayAngle(double _azimuth, double _yaw) const
{
  if (cfg_.north_up)
    return WrapAngle(_azimuth + _yaw - kPi / 2.0);
  return WrapAngle(_azimuth);
}

/////////////////////////////////////////////////
void PpiRenderer::ToPixel(double _range, double _angle, double &_u,
                          double &_v) const
{
  const double rPx = _range / mPerPx_;
  _u = center_ - rPx * std::sin(_angle);
  _v = center_ - rPx * std::cos(_angle);
}

/////////////////////////////////////////////////
void PpiRenderer::DrawRadial(RgbImage &_img, double _angle,
                             const uint8_t _rgb[3], bool _thick) const
{
  const double su = -std::sin(_angle);
  const double sv = -std::cos(_angle);
  for (double s = 0.0; s <= radiusPx_; s += 0.5)
  {
    const int u = static_cast<int>(std::lround(center_ + s * su));
    const int v = static_cast<int>(std::lround(center_ + s * sv));
    SetPixel(_img, u, v, _rgb);
    if (_thick)
    {
      SetPixel(_img, u + 1, v, _rgb);
      SetPixel(_img, u, v + 1, _rgb);
    }
  }
}

/////////////////////////////////////////////////
void PpiRenderer::DrawBox(RgbImage &_img, double _u, double _v, int _half,
                          const uint8_t _rgb[3]) const
{
  const int u0 = static_cast<int>(std::lround(_u)) - _half;
  const int u1 = static_cast<int>(std::lround(_u)) + _half;
  const int v0 = static_cast<int>(std::lround(_v)) - _half;
  const int v1 = static_cast<int>(std::lround(_v)) + _half;
  for (int u = u0; u <= u1; ++u)
  {
    SetPixel(_img, u, v0, _rgb);
    SetPixel(_img, u, v1, _rgb);
  }
  for (int v = v0; v <= v1; ++v)
  {
    SetPixel(_img, u0, v, _rgb);
    SetPixel(_img, u1, v, _rgb);
  }
}

/////////////////////////////////////////////////
void PpiRenderer::Render(const DisplayBuffer &_buffer, double _time,
                         double _antennaAzimuth, const Pose &_pose,
                         const std::vector<Target> &_targets,
                         RgbImage &_out) const
{
  _out.Resize(size_, size_);
  const double maxAge = 1.5 * scanPeriod_;
  const size_t npx = cellLut_.size();
  pixLevel_.assign(npx, 0u);
  pixAge_.assign(npx, 0.0f);

  // 1) Gather: each pixel samples the polar cell under its centre. This
  //    fills targets that are larger than a pixel.
  for (size_t idx = 0; idx < npx; ++idx)
  {
    const int32_t cell = cellLut_[idx];
    if (cell < 0)
      continue;
    const int slot = cell / bins_;
    const int bin = cell - slot * bins_;
    const uint8_t level = _buffer.Level(slot, bin);
    if (level == 0)
      continue;
    const double age = _time - _buffer.Time(slot);
    if (age > maxAge)
      continue;
    pixLevel_[idx] = level;
    pixAge_[idx] = static_cast<float>(age);
  }

  // 2) Splat: every echo cell paints the pixel under its centre, so echoes
  //    smaller than a pixel (point targets, thin range cells) never vanish
  //    between pixel centres.
  const double displayPx = cfg_.EffectiveDisplayRange() / mPerPx_;
  for (int s = 0; s < _buffer.Slots(); ++s)
  {
    const auto &echoes = _buffer.SlotEchoes(s);
    if (echoes.empty())
      continue;
    const double age = _time - _buffer.Time(s);
    if (age > maxAge)
      continue;
    const double si = slotSin_[static_cast<size_t>(s)];
    const double co = slotCos_[static_cast<size_t>(s)];
    for (const auto &e : echoes)
    {
      const double rPx = binPx_[static_cast<size_t>(e.first)];
      if (rPx > displayPx)
        continue;
      const int u = static_cast<int>(std::lround(center_ - rPx * si));
      const int v = static_cast<int>(std::lround(center_ - rPx * co));
      if (u < 0 || v < 0 || u >= size_ || v >= size_)
        continue;
      const size_t idx = static_cast<size_t>(v) * static_cast<size_t>(size_) +
                         static_cast<size_t>(u);
      if (e.second > pixLevel_[idx])
      {
        pixLevel_[idx] = e.second;
        pixAge_[idx] = static_cast<float>(age);
      }
    }
  }

  // Echo expansion (like the EXP key of marine radars): grow every echo by
  // one pixel so small targets stay visible at long display ranges.
  if (cfg_.echo_expansion)
  {
    expandLevel_ = pixLevel_;
    expandAge_ = pixAge_;
    for (int v = 0; v < size_; ++v)
    {
      for (int u = 0; u < size_; ++u)
      {
        const size_t idx = static_cast<size_t>(v) *
                           static_cast<size_t>(size_) + static_cast<size_t>(u);
        const uint8_t level = pixLevel_[idx];
        if (level == 0)
          continue;
        const int nb[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (const auto &d : nb)
        {
          const int uu = u + d[0];
          const int vv = v + d[1];
          if (uu < 0 || vv < 0 || uu >= size_ || vv >= size_)
            continue;
          const size_t j = static_cast<size_t>(vv) *
                           static_cast<size_t>(size_) +
                           static_cast<size_t>(uu);
          if (overlay_[j] == 0 || level <= expandLevel_[j])
            continue;
          expandLevel_[j] = level;
          expandAge_[j] = pixAge_[idx];
        }
      }
    }
    pixLevel_.swap(expandLevel_);
    pixAge_.swap(expandAge_);
  }

  // 3) Background, overlays and the heading line (drawn under the video so
  //    a target dead ahead stays visible).
  for (size_t idx = 0; idx < npx; ++idx)
  {
    uint8_t *px = &_out.data[idx * 3];
    const uint8_t *base = kOutside;
    switch (overlay_[idx])
    {
      case 1: base = kInside; break;
      case 2: base = kRing; break;
      case 3: base = kBearingLine; break;
      default: break;
    }
    px[0] = base[0];
    px[1] = base[1];
    px[2] = base[2];
  }
  const double yaw = _pose.Yaw();
  DrawRadial(_out, DisplayAngle(0.0, yaw), kHeading, false);

  // 4) Echo video with afterglow.
  for (size_t idx = 0; idx < npx; ++idx)
  {
    const uint8_t level = pixLevel_[idx];
    if (level == 0)
      continue;
    const double bright = 1.0 - cfg_.afterglow *
        std::min(1.0, std::max(0.0, pixAge_[idx] / scanPeriod_));
    const uint8_t *c = &palette_[static_cast<size_t>(level) * 3];
    uint8_t *px = &_out.data[idx * 3];
    px[0] = static_cast<uint8_t>(std::lround(c[0] * bright));
    px[1] = static_cast<uint8_t>(std::lround(c[1] * bright));
    px[2] = static_cast<uint8_t>(std::lround(c[2] * bright));
  }

  // 5) Sweep line at the current antenna azimuth.
  DrawRadial(_out, DisplayAngle(_antennaAzimuth, yaw), kSweep, true);

  // 6) Markers for the targets of the last completed scan.
  if (cfg_.draw_target_markers)
  {
    for (const Target &t : _targets)
    {
      double range = 0.0;
      double angle = 0.0;
      if (cfg_.north_up)
      {
        const Vec3 d = t.position_world - _pose.position;
        range = std::hypot(d.x, d.y);
        angle = WrapAngle(std::atan2(d.y, d.x) - kPi / 2.0);
      }
      else
      {
        const Vec3 p = _pose.InverseTransform(t.position_world);
        range = std::hypot(p.x, p.y);
        angle = std::atan2(p.y, p.x);
      }
      if (range > cfg_.EffectiveDisplayRange())
        continue;
      double u = 0.0, v = 0.0;
      ToPixel(range, angle, u, v);
      const double extentM =
          std::max(t.range_extent, t.range * t.bearing_extent);
      int half = static_cast<int>(std::lround(extentM / mPerPx_ / 2.0)) + 3;
      half = std::min(24, std::max(4, half));
      DrawBox(_out, u, v, half, kMarker);
    }
  }

  // Own-ship / radar position.
  const int c = static_cast<int>(std::lround(center_));
  for (int dv = -1; dv <= 1; ++dv)
    for (int du = -1; du <= 1; ++du)
      SetPixel(_out, c + du, c + dv, kCenter);
}

}  // namespace marine_radar
