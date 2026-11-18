// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Component 2 (radar side): turns geometric ray hits into radar echoes.
//
// For every spoke the echo model combines the raw hits of all rays inside the
// horizontal main lobe (neighbouring azimuths), applies a simplified radar
// equation and detection threshold, quantises range into bins and adds the
// configured noise, missed echoes and false echoes. This is what turns a
// geometric ray cast into radar video: point targets are smeared over the
// beam width, weak / grazing / distant returns fade below the threshold,
// and only the first surface along each ray is seen (occlusion).

#ifndef GZ_SENSORS_MARINE_RADAR_ECHOMODEL_HH_
#define GZ_SENSORS_MARINE_RADAR_ECHOMODEL_HH_

#include <cstdint>
#include <random>
#include <utility>
#include <vector>

#include "marine_radar/RadarConfig.hh"
#include "marine_radar/ScanGeometry.hh"
#include "marine_radar/Types.hh"

namespace marine_radar
{

/// \brief One geometric hit of one ray.
struct RawHit
{
  double range{0.0};          ///< [m] distance from radar origin to the hit.
  double elevation{0.0};      ///< [rad] elevation of the hit seen from the
                              ///< antenna, in the radar frame.
  double cos_incidence{1.0};  ///< |cos| of angle between ray and normal.
  double world_z{0.0};        ///< [m] world height of the hit point.
};

/// \brief All hits of the rays cast for one spoke azimuth.
struct RawSpoke
{
  int64_t g{0};
  std::vector<RawHit> hits;
};

/// \brief One detected range cell of one spoke.
struct Echo
{
  int bin{0};
  float snr_db{0.0f};       ///< Simulated signal-to-noise ratio [dB].
  float range{0.0f};        ///< [m] measured (noisy) slant range.
  float elevation{0.0f};    ///< [rad] beam layer elevation.
  bool false_alarm{false};  ///< True for injected false echoes.
};

/// \brief The processed video of one spoke.
struct ProcessedSpoke
{
  bool valid{false};
  int64_t g{0};
  int azimuth_index{0};
  double azimuth{0.0};   ///< [rad] beam-axis azimuth in the radar frame.
  double time{0.0};      ///< [s] simulation time of the spoke.
  Pose pose;             ///< Radar pose in the world at that time.
  std::vector<Echo> echoes;  ///< Sorted by bin, at most one per bin.
};

class EchoModel
{
 public:
  EchoModel(const RadarConfig &_cfg, const BeamModel &_beam);

  /// \brief Form the echoes of one spoke.
  /// \param[in] _neighbours raw spokes inside the main lobe, each paired
  ///   with its angular offset from the beam axis [rad].
  /// \param[in,out] _rng random generator (shared for reproducibility).
  std::vector<Echo> Form(
      const std::vector<std::pair<double, const RawSpoke *>> &_neighbours,
      std::mt19937 &_rng) const;

  /// \brief SNR [dB] of a hit, before random fluctuation.
  double HitSnrDb(const RawHit &_hit, double _horizontalOffset) const;

  /// \brief Map an SNR to an 8-bit display level (0 = no echo).
  uint8_t DisplayLevel(double _snrDb) const;

 private:
  RadarConfig cfg_;
  BeamModel beam_;
  int numBins_;
  int numSpokes_;
};

}  // namespace marine_radar

#endif  // GZ_SENSORS_MARINE_RADAR_ECHOMODEL_HH_
