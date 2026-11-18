// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Facade that ties the radar pipeline together. It is independent of the
// geometry back end: a caller (the Gazebo system plugin, or a unit test)
//   1. asks PlanRays(t) which rays must be cast now,
//   2. casts them in its environment and passes the hits to SetRayResults(),
//   3. calls Process(t, pose), which forms spokes, and returns a ScanResult
//      every time the antenna completes a revolution (or FOV sector),
//   4. calls RenderImage() whenever a PPI image is wanted.
// Image and detections are produced from the same echo video, so they are
// always consistent with each other.

#ifndef GZ_SENSORS_MARINE_RADAR_RADARPROCESSOR_HH_
#define GZ_SENSORS_MARINE_RADAR_RADARPROCESSOR_HH_

#include <cstdint>
#include <map>
#include <random>
#include <vector>

#include "marine_radar/EchoModel.hh"
#include "marine_radar/PpiRenderer.hh"
#include "marine_radar/RadarConfig.hh"
#include "marine_radar/ScanGeometry.hh"
#include "marine_radar/TargetExtraction.hh"
#include "marine_radar/Types.hh"

namespace marine_radar
{

/// \brief A ray to cast, expressed in the radar frame. Rays are horizontal
/// in the world (one per height slice) and start range_min away from the
/// antenna along the spoke's horizontal direction.
struct RayRequest
{
  int64_t g{0};          ///< Global sweep index of the spoke.
  int slice{0};          ///< Index into RadarConfig::slice_heights.
  double azimuth{0.0};   ///< [rad] spoke azimuth in the radar frame.
  Vec3 start;            ///< Ray start in the radar frame.
  Vec3 end;              ///< Ray end in the radar frame.
};

/// \brief Result of one ray cast, in the radar frame. A miss is signalled by
/// a non-finite fraction (Gazebo returns NaN).
struct RayResult
{
  double fraction{0.0};
  Vec3 point;
  Vec3 normal;
};

/// \brief Output of one completed scan.
struct ScanResult
{
  uint64_t scan_index{0};
  double stamp{0.0};     ///< [s] time of the last spoke of the scan.
  Pose pose;             ///< Radar pose at the stamp.
  std::vector<Target> targets;
  std::vector<ReturnPoint> returns;
};

/// \brief Hits whose incidence is closer than ~1.15 deg to grazing
/// (|cos| below this value) are ignored. See SetRayResults().
constexpr double kMinCosIncidence = 0.02;

struct ProcessorStats
{
  uint64_t rays_planned{0};
  uint64_t ray_hits{0};
  uint64_t grazing_hits_rejected{0};
  uint64_t spokes_processed{0};
  uint64_t spokes_skipped{0};
  uint64_t scans_completed{0};
};

class RadarProcessor
{
 public:
  /// \throws std::invalid_argument if the configuration is invalid.
  explicit RadarProcessor(const RadarConfig &_cfg);

  const RadarConfig &Config() const { return cfg_; }

  /// \brief Rays that must be cast for simulation time _t.
  /// \param[in] _pose current radar pose in the world (used to level the
  ///   height slices and to express the rays in the radar frame).
  const std::vector<RayRequest> &PlanRays(double _t, const Pose &_pose);

  /// \brief Results for the rays returned by the last PlanRays() call, in
  /// the same order.
  /// \param[in] _pose radar pose in the world when the rays were cast.
  void SetRayResults(const std::vector<RayResult> &_results,
                     const Pose &_pose);

  /// \brief Form every spoke due up to time _t. Returns the completed scans.
  std::vector<ScanResult> Process(double _t, const Pose &_pose);

  /// \brief Render the PPI image for time _t.
  void RenderImage(double _t, const Pose &_pose, RgbImage &_out) const;

  double AntennaAzimuth(double _t) const;
  const std::vector<Target> &LastTargets() const { return lastTargets_; }
  const ProcessorStats &Stats() const { return stats_; }
  const AntennaScheduler &Scheduler() const { return sched_; }
  const DisplayBuffer &Display() const { return display_; }

  /// \brief Forget all state (e.g. after a world reset).
  void Reset();

 private:
  void Start(double _t);
  void FinishScan(std::vector<ScanResult> &_out);

  RadarConfig cfg_;
  AntennaScheduler sched_;
  BeamModel beam_;
  EchoModel echo_;
  TargetExtractor extractor_;
  TargetTracker tracker_;
  PpiRenderer renderer_;
  DisplayBuffer display_;
  std::mt19937 rng_;

  bool started_{false};
  int64_t nextCast_{0};
  int64_t nextProcess_{0};
  int64_t currentRev_{0};
  std::vector<RayRequest> plan_;
  std::map<int64_t, RawSpoke> raw_;
  std::vector<ProcessedSpoke> scan_;
  std::vector<Target> lastTargets_;
  uint64_t scanCount_{0};
  ProcessorStats stats_;
};

}  // namespace marine_radar

#endif  // GZ_SENSORS_MARINE_RADAR_RADARPROCESSOR_HH_
