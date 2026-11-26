// Copyright 2026 marine_radar contributors. Apache-2.0.

#include "marine_radar/EchoModel.hh"

#include <algorithm>
#include <cmath>

namespace marine_radar
{

/////////////////////////////////////////////////
EchoModel::EchoModel(const RadarConfig &_cfg, const BeamModel &_beam)
  : cfg_(_cfg), beam_(_beam), numBins_(_cfg.NumBins()),
    numSpokes_(_cfg.NumSpokes())
{
}

/////////////////////////////////////////////////
double EchoModel::HitSnrDb(const RawHit &_hit, double _offset) const
{
  // Simplified radar equation for an extended surface element:
  //   SNR = SNR_ref * |cos(incidence)| * (R_ref / R)^4 * G_h * G_v
  const double cosInc = std::max(std::abs(_hit.cos_incidence), 1e-3);
  const double r = std::max(_hit.range, 1.0);
  return cfg_.snr_reference_db + 10.0 * std::log10(cosInc) -
         40.0 * std::log10(r / cfg_.reference_range) +
         beam_.HorizontalGainDb(_offset) +
         beam_.VerticalGainDb(_hit.elevation);
}

/////////////////////////////////////////////////
uint8_t EchoModel::DisplayLevel(double _snrDb) const
{
  if (!(_snrDb >= cfg_.detection_threshold_db))
    return 0u;
  double q = (_snrDb - cfg_.detection_threshold_db) /
      cfg_.display_dynamic_range_db;
  q = std::min(1.0, std::max(0.0, q));
  return static_cast<uint8_t>(1 + std::lround(q * 254.0));
}

/////////////////////////////////////////////////
std::vector<Echo> EchoModel::Form(
    const std::vector<std::pair<double, const RawSpoke *>> &_neighbours,
    std::mt19937 &_rng) const
{
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::vector<Echo> cells;

  for (const auto &nb : _neighbours)
  {
    if (nb.second == nullptr)
      continue;
    for (const RawHit &hit : nb.second->hits)
    {
      // Radar energy does not penetrate the sea surface.
      if (cfg_.sea_level_filter && hit.world_z < cfg_.sea_level)
        continue;
      double snr = HitSnrDb(hit, nb.first);
      if (cfg_.intensity_noise_stddev_db > 0.0)
        snr += cfg_.intensity_noise_stddev_db * gauss(_rng);
      double r = hit.range;
      if (cfg_.range_noise_stddev > 0.0)
        r += cfg_.range_noise_stddev * gauss(_rng);
      if (r < cfg_.range_min || r >= cfg_.range_max)
        continue;
      int bin = static_cast<int>(
          std::floor((r - cfg_.range_min) / cfg_.range_resolution));
      bin = std::min(std::max(bin, 0), numBins_ - 1);
      Echo e;
      e.bin = bin;
      e.snr_db = static_cast<float>(snr);
      e.range = static_cast<float>(r);
      e.elevation = static_cast<float>(hit.elevation);
      cells.push_back(e);
    }
  }

  // One echo per range bin: the returns of all surfaces in the same
  // resolution cell add up in power (bigger / taller objects return more
  // energy). Range and elevation are taken from the strongest contribution.
  std::sort(cells.begin(), cells.end(), [](const Echo &_a, const Echo &_b)
  {
    return _a.bin < _b.bin || (_a.bin == _b.bin && _a.snr_db > _b.snr_db);
  });
  std::vector<Echo> out;
  out.reserve(cells.size());
  size_t i0 = 0;
  while (i0 < cells.size())
  {
    size_t i1 = i0;
    double power = 0.0;
    // cells[i1].bin == cells[i0].bin always holds on the first iteration
    // (i1 == i0), so this loop runs at least once and power ends up > 0.
    do
    {
      power += std::pow(10.0, cells[i1].snr_db / 10.0);
      ++i1;
    } while (i1 < cells.size() && cells[i1].bin == cells[i0].bin);
    const double snr = 10.0 * std::log10(power);
    if (snr >= cfg_.detection_threshold_db)
    {
      Echo e = cells[i0];  // strongest contribution (sorted first)
      e.snr_db = static_cast<float>(snr);
      out.push_back(e);
    }
    i0 = i1;
  }

  // Missed echoes: each contiguous echo segment is received with the
  // configured probability.
  if (cfg_.echo_detection_probability < 1.0 && !out.empty())
  {
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    std::vector<Echo> kept;
    kept.reserve(out.size());
    size_t i = 0;
    while (i < out.size())
    {
      size_t j = i + 1;
      while (j < out.size() && out[j].bin - out[j - 1].bin <= 1)
        ++j;
      if (uni(_rng) < cfg_.echo_detection_probability)
        kept.insert(kept.end(), out.begin() + static_cast<std::ptrdiff_t>(i),
                    out.begin() + static_cast<std::ptrdiff_t>(j));
      i = j;
    }
    out.swap(kept);
  }

  // False echoes (thermal noise / interference crossing the threshold).
  if (cfg_.false_alarm_rate > 0.0)
  {
    std::poisson_distribution<int> poisson(
        cfg_.false_alarm_rate / static_cast<double>(numSpokes_));
    const int k = poisson(_rng);
    if (k > 0)
    {
      std::uniform_int_distribution<int> binDist(0, numBins_ - 1);
      std::exponential_distribution<double> excess(1.0 / 3.0);
      for (int n = 0; n < k; ++n)
      {
        Echo fa;
        fa.bin = binDist(_rng);
        fa.snr_db = static_cast<float>(
            cfg_.detection_threshold_db + excess(_rng));
        fa.range = static_cast<float>(cfg_.BinCenterRange(fa.bin));
        fa.elevation = 0.0f;
        fa.false_alarm = true;
        auto it = std::lower_bound(out.begin(), out.end(), fa.bin,
            [](const Echo &_e, int _b) { return _e.bin < _b; });
        if (it != out.end() && it->bin == fa.bin)
        {
          if (fa.snr_db > it->snr_db)
            *it = fa;
        }
        else
        {
          out.insert(it, fa);
        }
      }
    }
  }
  return out;
}

}  // namespace marine_radar
