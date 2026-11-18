// Copyright 2026 marine_radar contributors. Apache-2.0.

#include "marine_radar/TargetExtraction.hh"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <utility>

#include "marine_radar/ScanGeometry.hh"

namespace marine_radar
{
namespace
{

class UnionFind
{
 public:
  explicit UnionFind(size_t _n) : parent_(_n) {
    std::iota(parent_.begin(), parent_.end(), 0u);
  }
  size_t Find(size_t _x)
  {
    while (parent_[_x] != _x)
    {
      parent_[_x] = parent_[parent_[_x]];
      _x = parent_[_x];
    }
    return _x;
  }
  void Union(size_t _a, size_t _b)
  {
    _a = Find(_a);
    _b = Find(_b);
    if (_a == _b)
      return;
    // Keep the smaller index as root for deterministic ordering.
    if (_a < _b)
      parent_[_b] = _a;
    else
      parent_[_a] = _b;
  }

 private:
  std::vector<size_t> parent_;
};

}  // namespace

/////////////////////////////////////////////////
TargetExtractor::TargetExtractor(const RadarConfig &_cfg) : cfg_(_cfg)
{
}

/////////////////////////////////////////////////
std::vector<Target> TargetExtractor::Extract(
    const std::vector<ProcessedSpoke> &_scan, std::mt19937 &_rng) const
{
  const int n = static_cast<int>(_scan.size());
  std::vector<size_t> offset(static_cast<size_t>(n) + 1, 0u);
  for (int p = 0; p < n; ++p)
  {
    const size_t cnt = _scan[static_cast<size_t>(p)].valid ?
        _scan[static_cast<size_t>(p)].echoes.size() : 0u;
    offset[static_cast<size_t>(p) + 1] = offset[static_cast<size_t>(p)] + cnt;
  }
  const size_t numNodes = offset.back();
  std::vector<Target> targets;
  if (numNodes == 0)
    return targets;

  UnionFind uf(numNodes);
  const int binReach = 1 + cfg_.cluster_bin_gap;
  const int spokeReach = 1 + cfg_.cluster_spoke_gap;
  const bool wrap = cfg_.FullCircle();

  for (int p = 0; p < n; ++p)
  {
    const ProcessedSpoke &sp = _scan[static_cast<size_t>(p)];
    if (!sp.valid || sp.echoes.empty())
      continue;
    const size_t base = offset[static_cast<size_t>(p)];
    // Along range within the spoke.
    for (size_t i = 1; i < sp.echoes.size(); ++i)
    {
      if (sp.echoes[i].bin - sp.echoes[i - 1].bin <= binReach)
        uf.Union(base + i - 1, base + i);
    }
    // Across neighbouring spokes.
    for (int d = 1; d <= spokeReach; ++d)
    {
      int q = p + d;
      if (q >= n)
      {
        if (!wrap)
          break;
        q -= n;
      }
      if (q == p)
        continue;
      const ProcessedSpoke &sq = _scan[static_cast<size_t>(q)];
      if (!sq.valid || sq.echoes.empty())
        continue;
      const size_t baseQ = offset[static_cast<size_t>(q)];
      for (size_t i = 0; i < sp.echoes.size(); ++i)
      {
        const int b = sp.echoes[i].bin;
        auto it = std::lower_bound(sq.echoes.begin(), sq.echoes.end(),
            b - binReach, [](const Echo &_e, int _v) { return _e.bin < _v; });
        for (; it != sq.echoes.end() && it->bin <= b + binReach; ++it)
        {
          uf.Union(base + i,
                   baseQ + static_cast<size_t>(it - sq.echoes.begin()));
        }
      }
    }
  }

  // Gather clusters in order of their first (root) node: deterministic.
  std::unordered_map<size_t, size_t> rootToCluster;
  std::vector<std::vector<std::pair<int, size_t>>> clusters;  // (spoke, idx)
  for (int p = 0; p < n; ++p)
  {
    const ProcessedSpoke &sp = _scan[static_cast<size_t>(p)];
    if (!sp.valid)
      continue;
    for (size_t i = 0; i < sp.echoes.size(); ++i)
    {
      const size_t root = uf.Find(offset[static_cast<size_t>(p)] + i);
      auto it = rootToCluster.find(root);
      if (it == rootToCluster.end())
      {
        it = rootToCluster.emplace(root, clusters.size()).first;
        clusters.emplace_back();
      }
      clusters[it->second].emplace_back(p, i);
    }
  }

  std::normal_distribution<double> gauss(0.0, 1.0);
  const double spacing = cfg_.SpokeSpacing();
  for (const auto &cl : clusters)
  {
    if (static_cast<int>(cl.size()) < cfg_.min_target_cells)
      continue;
    double sw = 0.0, ss = 0.0, sc = 0.0, sr = 0.0, se = 0.0;
    double minR = std::numeric_limits<double>::max();
    double maxR = std::numeric_limits<double>::lowest();
    double peak = std::numeric_limits<double>::lowest();
    int peakSpoke = cl.front().first;
    uint32_t falseCells = 0;
    for (const auto &cell : cl)
    {
      const ProcessedSpoke &sp = _scan[static_cast<size_t>(cell.first)];
      const Echo &e = sp.echoes[cell.second];
      // Weight by dB above threshold: robust centroid, not peak-dominated.
      const double w = std::max(
          1.0, static_cast<double>(e.snr_db) - cfg_.detection_threshold_db +
          1.0);
      sw += w;
      ss += w * std::sin(sp.azimuth);
      sc += w * std::cos(sp.azimuth);
      sr += w * e.range;
      se += w * e.elevation;
      minR = std::min(minR, static_cast<double>(e.range));
      maxR = std::max(maxR, static_cast<double>(e.range));
      if (e.snr_db > peak)
      {
        peak = e.snr_db;
        peakSpoke = cell.first;
      }
      if (e.false_alarm)
        ++falseCells;
    }
    Target t;
    t.bearing = std::atan2(ss, sc);
    double minOff = std::numeric_limits<double>::max();
    double maxOff = std::numeric_limits<double>::lowest();
    for (const auto &cell : cl)
    {
      const double off = WrapAngle(
          _scan[static_cast<size_t>(cell.first)].azimuth - t.bearing);
      minOff = std::min(minOff, off);
      maxOff = std::max(maxOff, off);
    }
    t.bearing_extent = (maxOff - minOff) + spacing;
    if (cfg_.bearing_noise_stddev > 0.0)
      t.bearing = WrapAngle(t.bearing + cfg_.bearing_noise_stddev *
                            gauss(_rng));
    t.range = sr / sw;
    t.elevation = se / sw;
    t.min_range = minR;
    t.range_extent = (maxR - minR) + cfg_.range_resolution;
    t.intensity_db = peak;
    t.num_cells = static_cast<uint32_t>(cl.size());
    t.num_false_cells = falseCells;
    const ProcessedSpoke &ps = _scan[static_cast<size_t>(peakSpoke)];
    t.time = ps.time;
    t.position_radar = BeamModel::Direction(t.bearing, t.elevation) * t.range;
    t.position_world = ps.pose.Transform(t.position_radar);
    targets.push_back(t);
  }

  std::sort(targets.begin(), targets.end(),
            [](const Target &_a, const Target &_b)
            { return _a.range < _b.range; });
  return targets;
}

/////////////////////////////////////////////////
std::vector<ReturnPoint> TargetExtractor::Returns(
    const std::vector<ProcessedSpoke> &_scan) const
{
  std::vector<ReturnPoint> out;
  for (const ProcessedSpoke &sp : _scan)
  {
    if (!sp.valid)
      continue;
    for (const Echo &e : sp.echoes)
    {
      ReturnPoint rp;
      rp.range = e.range;
      rp.bearing = sp.azimuth;
      rp.elevation = e.elevation;
      rp.intensity_db = e.snr_db;
      rp.time = sp.time;
      rp.position_radar =
          BeamModel::Direction(sp.azimuth, e.elevation) * rp.range;
      rp.position_world = sp.pose.Transform(rp.position_radar);
      out.push_back(rp);
    }
  }
  return out;
}

/////////////////////////////////////////////////
TargetTracker::TargetTracker(const RadarConfig &_cfg)
  : gate_(_cfg.association_gate), coast_(_cfg.track_coast_scans)
{
}

/////////////////////////////////////////////////
void TargetTracker::Reset()
{
  tracks_.clear();
  nextId_ = 1;
}

/////////////////////////////////////////////////
void TargetTracker::Associate(std::vector<Target> &_targets)
{
  struct Pair
  {
    double d;
    size_t target;
    size_t track;
  };
  std::vector<Pair> pairs;
  for (size_t i = 0; i < _targets.size(); ++i)
  {
    for (size_t j = 0; j < tracks_.size(); ++j)
    {
      const double d =
          (_targets[i].position_world - tracks_[j].position).Length();
      if (d <= gate_)
        pairs.push_back({d, i, j});
    }
  }
  std::sort(pairs.begin(), pairs.end(), [](const Pair &_a, const Pair &_b)
  {
    if (_a.d != _b.d)
      return _a.d < _b.d;
    if (_a.target != _b.target)
      return _a.target < _b.target;
    return _a.track < _b.track;
  });

  std::vector<bool> targetDone(_targets.size(), false);
  std::vector<bool> trackDone(tracks_.size(), false);
  for (const Pair &p : pairs)
  {
    if (targetDone[p.target] || trackDone[p.track])
      continue;
    targetDone[p.target] = true;
    trackDone[p.track] = true;
    Track &tr = tracks_[p.track];
    tr.age += 1;
    tr.missed = 0;
    tr.position = _targets[p.target].position_world;
    _targets[p.target].id = tr.id;
    _targets[p.target].age = tr.age;
  }

  std::vector<Track> next;
  next.reserve(tracks_.size() + _targets.size());
  for (size_t j = 0; j < tracks_.size(); ++j)
  {
    Track tr = tracks_[j];
    if (!trackDone[j])
    {
      tr.missed += 1;
      tr.age = 0;
      if (tr.missed > coast_)
        continue;
    }
    next.push_back(tr);
  }
  for (size_t i = 0; i < _targets.size(); ++i)
  {
    if (targetDone[i])
      continue;
    if (nextId_ == 0)
      nextId_ = 1;
    Track tr{nextId_++, 1u, _targets[i].position_world, 0};
    _targets[i].id = tr.id;
    _targets[i].age = 1;
    next.push_back(tr);
  }
  tracks_.swap(next);
}

}  // namespace marine_radar
