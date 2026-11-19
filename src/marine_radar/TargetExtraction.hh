// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Component 4 of the pipeline: target extraction and identification.
//
// After every completed scan, echo cells are grouped into targets ("plots")
// with connected-component labelling on the polar spoke x range-bin grid,
// including the wrap between the last and first spoke of a 360 degree scan.
// A large object (another ship, a pier, a shoreline section) therefore yields
// one target instead of one detection per ray. The TargetTracker then gives
// targets persistent IDs by nearest-neighbour association in the world frame.

#ifndef GZ_SENSORS_MARINE_RADAR_TARGETEXTRACTION_HH_
#define GZ_SENSORS_MARINE_RADAR_TARGETEXTRACTION_HH_

#include <cstdint>
#include <random>
#include <vector>

#include "marine_radar/EchoModel.hh"
#include "marine_radar/RadarConfig.hh"
#include "marine_radar/Types.hh"

namespace marine_radar
{

/// \brief One extracted radar target.
struct Target
{
  uint32_t id{0};            ///< Persistent ID (assigned by TargetTracker).
  uint32_t age{0};           ///< Consecutive scans associated (1 = new).
  double range{0.0};         ///< [m] intensity-weighted centroid range.
  double bearing{0.0};       ///< [rad] REP-103 azimuth in the radar frame.
  double elevation{0.0};     ///< [rad] weighted mean beam-layer elevation.
  Vec3 position_radar;       ///< Centroid in the radar frame.
  Vec3 position_world;       ///< Centroid in the world frame.
  double intensity_db{0.0};  ///< Peak SNR [dB].
  double min_range{0.0};     ///< [m] closest echo of the target.
  double range_extent{0.0};  ///< [m]
  double bearing_extent{0.0};  ///< [rad]
  uint32_t num_cells{0};
  uint32_t num_false_cells{0};  ///< Cells that came from false echoes.
  double time{0.0};          ///< [s] time the strongest cell was observed.
};

/// \brief One echo cell expressed as a point (for point-cloud output).
struct ReturnPoint
{
  Vec3 position_radar;
  Vec3 position_world;
  double range{0.0};
  double bearing{0.0};
  double elevation{0.0};
  float intensity_db{0.0f};
  double time{0.0};
};

class TargetExtractor
{
 public:
  explicit TargetExtractor(const RadarConfig &_cfg);

  /// \brief Extract targets from one scan.
  /// \param[in] _scan processed spokes indexed by sweep position. Invalid
  ///   entries (spokes not yet observed) are ignored.
  /// \param[in,out] _rng random generator (bearing noise).
  std::vector<Target> Extract(const std::vector<ProcessedSpoke> &_scan,
                              std::mt19937 &_rng) const;

  /// \brief Convert all echo cells of a scan into points.
  std::vector<ReturnPoint> Returns(
      const std::vector<ProcessedSpoke> &_scan) const;

 private:
  RadarConfig cfg_;
};

class TargetTracker
{
 public:
  explicit TargetTracker(const RadarConfig &_cfg);

  /// \brief Assign ids/ages to the targets of a new scan.
  void Associate(std::vector<Target> &_targets);

  void Reset();

 private:
  struct Track
  {
    uint32_t id;
    uint32_t age;
    Vec3 position;
    int missed;
  };
  double gate_;
  int coast_;
  uint32_t nextId_{1};
  std::vector<Track> tracks_;
};

}  // namespace marine_radar

#endif  // GZ_SENSORS_MARINE_RADAR_TARGETEXTRACTION_HH_
