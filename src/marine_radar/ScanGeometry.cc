// Copyright 2026 marine_radar contributors. Apache-2.0.

#include "marine_radar/ScanGeometry.hh"

#include <algorithm>
#include <cmath>

namespace marine_radar
{
namespace
{
int64_t FloorDiv(int64_t _a, int64_t _b)
{
  int64_t q = _a / _b;
  if ((_a % _b != 0) && ((_a < 0) != (_b < 0)))
    --q;
  return q;
}
}  // namespace

/////////////////////////////////////////////////
AntennaScheduler::AntennaScheduler(const RadarConfig &_cfg)
  : cfg_(_cfg),
    numSpokes_(_cfg.NumSpokes()),
    spacing_(_cfg.SpokeSpacing()),
    period_(_cfg.ScanPeriod()),
    dwell_(_cfg.SpokeDwell())
{
}

/////////////////////////////////////////////////
int64_t AntennaScheduler::Revolution(int64_t _g) const
{
  return FloorDiv(_g, numSpokes_);
}

/////////////////////////////////////////////////
int AntennaScheduler::SweepPosition(int64_t _g) const
{
  return static_cast<int>(_g - Revolution(_g) * numSpokes_);
}

/////////////////////////////////////////////////
int AntennaScheduler::AzimuthIndex(int64_t _g) const
{
  return numSpokes_ - 1 - SweepPosition(_g);
}

/////////////////////////////////////////////////
double AntennaScheduler::SpokeAzimuth(int64_t _g) const
{
  return cfg_.SpokeAzimuth(AzimuthIndex(_g));
}

/////////////////////////////////////////////////
double AntennaScheduler::DueTime(int64_t _g) const
{
  const int64_t rev = Revolution(_g);
  const int p = SweepPosition(_g);
  return (static_cast<double>(rev) +
          (static_cast<double>(p) + 0.5) * spacing_ / kTwoPi) * period_;
}

/////////////////////////////////////////////////
int64_t AntennaScheduler::LastDueIndex(double _t) const
{
  if (!std::isfinite(_t) || _t < DueTime(0))
    return -1;
  const double revs = _t / period_;
  const int64_t rev = static_cast<int64_t>(std::floor(revs));
  const double frac = revs - static_cast<double>(rev);
  const double pf = frac * kTwoPi / spacing_ - 0.5;
  int64_t g;
  if (pf < 0.0)
    g = rev * numSpokes_ - 1;
  else if (pf >= numSpokes_)
    g = rev * numSpokes_ + numSpokes_ - 1;
  else
    g = rev * numSpokes_ + static_cast<int64_t>(std::floor(pf));
  // Guard against floating point round-off at spoke boundaries.
  while (DueTime(g + 1) <= _t)
    ++g;
  while (g >= 0 && DueTime(g) > _t)
    --g;
  return g;
}

/////////////////////////////////////////////////
double AntennaScheduler::AntennaAzimuth(double _t) const
{
  const double revs = _t / period_;
  const double frac = revs - std::floor(revs);
  return WrapAngle(cfg_.horizontal_fov / 2.0 - frac * kTwoPi);
}

/////////////////////////////////////////////////
bool AntennaScheduler::Contiguous(int64_t _a, int64_t _b) const
{
  if (cfg_.FullCircle())
    return true;
  return Revolution(_a) == Revolution(_b);
}

/////////////////////////////////////////////////
BeamModel::BeamModel(const RadarConfig &_cfg)
  : bwH_(_cfg.beamwidth_horizontal),
    bwV_(_cfg.beamwidth_vertical)
{
  const double spacing = _cfg.SpokeSpacing();
  int h = static_cast<int>(std::ceil(bwH_ / spacing - 1e-9));
  h = std::max(0, std::min(h, _cfg.NumSpokes() / 2));
  halfBeamSpokes_ = h;
}

/////////////////////////////////////////////////
Vec3 BeamModel::Direction(double _azimuth, double _elevation)
{
  const double ce = std::cos(_elevation);
  return {ce * std::cos(_azimuth), ce * std::sin(_azimuth),
          std::sin(_elevation)};
}

/////////////////////////////////////////////////
double BeamModel::HorizontalGainDb(double _offset) const
{
  const double u = 2.0 * _offset / bwH_;
  return -3.0102999566 * u * u;
}

/////////////////////////////////////////////////
double BeamModel::VerticalGainDb(double _elevation) const
{
  const double u = 2.0 * _elevation / bwV_;
  return -3.0102999566 * u * u;
}

}  // namespace marine_radar
