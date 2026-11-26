// Copyright 2026 marine_radar contributors. Apache-2.0.

#include "marine_radar/RadarProcessor.hh"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace marine_radar
{
namespace
{
RadarConfig Checked(const RadarConfig &_cfg)
{
  const auto errors = _cfg.Validate();
  if (!errors.empty())
  {
    std::string msg = "invalid radar configuration:";
    for (const auto &e : errors)
      msg += "\n  - " + e;
    throw std::invalid_argument(msg);
  }
  return _cfg;
}

int DisplaySlots(const RadarConfig &_cfg)
{
  return static_cast<int>(std::lround(kTwoPi / _cfg.SpokeSpacing()));
}
}  // namespace

/////////////////////////////////////////////////
RadarProcessor::RadarProcessor(const RadarConfig &_cfg)
  : cfg_(Checked(_cfg)),
    sched_(cfg_),
    beam_(cfg_),
    echo_(cfg_, beam_),
    extractor_(cfg_),
    tracker_(cfg_),
    renderer_(cfg_),
    display_(DisplaySlots(cfg_), cfg_.NumBins()),
    rng_(cfg_.seed),
    scan_(static_cast<size_t>(cfg_.NumSpokes()))
{
}

/////////////////////////////////////////////////
void RadarProcessor::Reset()
{
  started_ = false;
  nextCast_ = 0;
  nextProcess_ = 0;
  currentRev_ = 0;
  plan_.clear();
  raw_.clear();
  for (auto &s : scan_)
    s = ProcessedSpoke();
  lastTargets_.clear();
  scanCount_ = 0;
  stats_ = ProcessorStats();
  tracker_.Reset();
  display_.Clear();
  rng_.seed(cfg_.seed);
}

/////////////////////////////////////////////////
double RadarProcessor::AntennaAzimuth(double _t) const
{
  return sched_.AntennaAzimuth(_t);
}

/////////////////////////////////////////////////
void RadarProcessor::Start(double _t)
{
  const int h = beam_.HalfBeamSpokes();
  nextProcess_ = sched_.LastDueIndex(_t) + 1;
  nextCast_ = std::max<int64_t>(0, nextProcess_ - h);
  currentRev_ = sched_.Revolution(nextProcess_);
  started_ = true;
}

/////////////////////////////////////////////////
const std::vector<RayRequest> &RadarProcessor::PlanRays(double _t,
                                                        const Pose &_pose)
{
  plan_.clear();
  if (!started_)
    Start(_t);

  const int n = sched_.SpokesPerScan();
  const int h = beam_.HalfBeamSpokes();
  const int64_t lastDue = sched_.LastDueIndex(_t);
  if (lastDue - nextProcess_ > 2 * static_cast<int64_t>(n))
  {
    // Simulation time jumped far ahead (e.g. very large step or the radar
    // was paused): skip forward instead of processing many stale spokes.
    stats_.spokes_skipped +=
        static_cast<uint64_t>(lastDue + 1 - nextProcess_);
    nextProcess_ = lastDue + 1;
    nextCast_ = std::max(nextCast_, nextProcess_ - h);
    raw_.clear();
    for (auto &s : scan_)
      s.valid = false;
    currentRev_ = sched_.Revolution(nextProcess_);
  }

  const double horizon = _t + h * sched_.Dwell() + 1e-12;
  const auto &heights = cfg_.slice_heights;
  while (sched_.DueTime(nextCast_) <= horizon)
  {
    const double az = sched_.SpokeAzimuth(nextCast_);
    // Horizontal direction of the spoke in the world.
    const Vec3 dw = _pose.orientation.Rotate({std::cos(az), std::sin(az), 0.0});
    const double hl = std::hypot(dw.x, dw.y);
    if (hl > 1e-6)
    {
      const double ux = dw.x / hl;
      const double uy = dw.y / hl;
      for (size_t k = 0; k < heights.size(); ++k)
      {
        const double z = cfg_.sea_level + heights[k];
        const Vec3 startW(_pose.position.x + cfg_.range_min * ux,
                          _pose.position.y + cfg_.range_min * uy, z);
        const Vec3 endW(_pose.position.x + cfg_.range_max * ux,
                        _pose.position.y + cfg_.range_max * uy, z);
        RayRequest r;
        r.g = nextCast_;
        r.slice = static_cast<int>(k);
        r.azimuth = az;
        r.start = _pose.InverseTransform(startW);
        r.end = _pose.InverseTransform(endW);
        plan_.push_back(r);
      }
    }
    raw_[nextCast_].g = nextCast_;
    ++nextCast_;
  }
  stats_.rays_planned += plan_.size();
  return plan_;
}

/////////////////////////////////////////////////
void RadarProcessor::SetRayResults(const std::vector<RayResult> &_results,
                                   const Pose &_pose)
{
  const size_t n = std::min(_results.size(), plan_.size());
  for (size_t i = 0; i < n; ++i)
  {
    const RayRequest &req = plan_[i];
    const RayResult &res = _results[i];
    if (!std::isfinite(res.fraction) || res.fraction < 0.0 ||
        res.fraction > 1.0 || !res.point.IsFinite())
    {
      continue;
    }
    Vec3 dir = req.end - req.start;
    const double len = dir.Length();
    if (len <= 0.0)
      continue;
    dir = dir * (1.0 / len);
    RawHit hit;
    hit.range = res.point.Length();
    hit.elevation =
        std::atan2(res.point.z, std::hypot(res.point.x, res.point.y));
    const double nl = res.normal.Length();
    hit.cos_incidence = (res.normal.IsFinite() && nl > 1e-9) ?
        std::abs(res.normal.Dot(dir)) / nl : 1.0;
    // Rays that graze an edge can produce phantom hits far behind the
    // object in Bullet's ray test (normal perpendicular to the ray). Such
    // hits would carry almost no energy anyway, so they are discarded.
    if (hit.cos_incidence < kMinCosIncidence)
    {
      ++stats_.grazing_hits_rejected;
      continue;
    }
    hit.world_z = _pose.Transform(res.point).z;
    RawSpoke &rs = raw_[req.g];
    rs.g = req.g;
    rs.hits.push_back(hit);
    ++stats_.ray_hits;
  }
}

/////////////////////////////////////////////////
std::vector<ScanResult> RadarProcessor::Process(double _t, const Pose &_pose)
{
  std::vector<ScanResult> out;
  if (!started_)
    Start(_t);

  const int n = sched_.SpokesPerScan();
  const int h = beam_.HalfBeamSpokes();
  const double spacing = cfg_.SpokeSpacing();
  const double yaw = _pose.Yaw();
  std::vector<std::pair<double, const RawSpoke *>> neighbours;
  std::vector<std::pair<int, uint8_t>> levels;

  while (sched_.DueTime(nextProcess_) <= _t)
  {
    const int64_t g = nextProcess_;
    const int64_t rev = sched_.Revolution(g);
    const int p = sched_.SweepPosition(g);
    if (rev != currentRev_)
    {
      // Spokes were skipped and the previous revolution never reached its
      // last spoke: publish what was collected.
      FinishScan(out);
      currentRev_ = rev;
    }

    neighbours.clear();
    for (int d = -h; d <= h; ++d)
    {
      const int64_t gn = g + d;
      if (gn < 0 || !sched_.Contiguous(gn, g))
        continue;
      auto it = raw_.find(gn);
      if (it != raw_.end())
        neighbours.emplace_back(d * spacing, &it->second);
    }

    ProcessedSpoke &ps = scan_[static_cast<size_t>(p)];
    ps.valid = true;
    ps.g = g;
    ps.azimuth_index = sched_.AzimuthIndex(g);
    ps.azimuth = sched_.SpokeAzimuth(g);
    ps.time = sched_.DueTime(g);
    ps.pose = _pose;
    ps.echoes = echo_.Form(neighbours, rng_);

    levels.clear();
    for (const Echo &e : ps.echoes)
      levels.emplace_back(e.bin, echo_.DisplayLevel(e.snr_db));
    display_.Paint(renderer_.DisplayAngle(ps.azimuth, yaw), ps.time, levels);

    ++stats_.spokes_processed;
    ++nextProcess_;
    if (p == n - 1)
    {
      FinishScan(out);
      currentRev_ = rev + 1;
    }
  }

  // Drop raw data no longer needed by any future spoke.
  const int64_t keepFrom = nextProcess_ - h - 1;
  raw_.erase(raw_.begin(), raw_.lower_bound(keepFrom));
  return out;
}

/////////////////////////////////////////////////
void RadarProcessor::FinishScan(std::vector<ScanResult> &_out)
{
  const ProcessedSpoke *last = nullptr;
  for (const auto &s : scan_)
  {
    if (s.valid && (last == nullptr || s.time > last->time))
      last = &s;
  }
  if (last == nullptr)
    return;

  ScanResult sr;
  sr.scan_index = scanCount_++;
  sr.stamp = last->time;
  sr.pose = last->pose;
  sr.targets = extractor_.Extract(scan_, rng_);
  tracker_.Associate(sr.targets);
  sr.returns = extractor_.Returns(scan_);
  lastTargets_ = sr.targets;
  ++stats_.scans_completed;
  for (auto &s : scan_)
  {
    s.valid = false;
    s.echoes.clear();
  }
  _out.push_back(std::move(sr));
}

/////////////////////////////////////////////////
void RadarProcessor::RenderImage(double _t, const Pose &_pose,
                                 RgbImage &_out) const
{
  renderer_.Render(display_, _t, sched_.AntennaAzimuth(_t), _pose,
                   lastTargets_, _out);
}

}  // namespace marine_radar
