// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Component 3 of the pipeline: angular scan generation.
//
// The antenna rotates clockwise seen from above (the usual marine radar
// convention), i.e. its REP-103 azimuth decreases with time. Within each
// revolution it transmits only inside horizontal_fov (sector blanking when
// the FOV is below 360 degrees). Every transmitted beam position is a
// "spoke". Spokes are numbered with a monotonic global sweep index
//   g = revolution * NumSpokes() + sweep_position,
// where sweep_position 0 is the first spoke illuminated in a revolution.
//
// The BeamModel provides the two-way antenna gain pattern and the extent of
// the horizontal main lobe in spokes.
//
// Ray sampling ("height slices"): a real marine radar beam is ~20 degrees
// tall, so beyond a few tens of metres it illuminates everything from the sea
// surface up. Sampling that beam with a fan of fixed-elevation rays from the
// mast would miss low objects (a 5-degree fan from 5 m passes over a 3 m buoy
// at 80 m). Instead, every spoke is sampled with gravity-levelled rays at
// fixed heights above sea level (slice_heights), starting range_min from the
// antenna along the spoke's horizontal direction. The vertical beam pattern
// is then applied from the true elevation of each hit as seen from the
// antenna, so low objects close to the ship still fall out of the beam and
// roll/pitch of the vessel still tilts the beam.

#ifndef GZ_SENSORS_MARINE_RADAR_SCANGEOMETRY_HH_
#define GZ_SENSORS_MARINE_RADAR_SCANGEOMETRY_HH_

#include <cstdint>
#include <vector>

#include "marine_radar/RadarConfig.hh"
#include "marine_radar/Types.hh"

namespace marine_radar
{

class AntennaScheduler
{
 public:
  explicit AntennaScheduler(const RadarConfig &_cfg);

  int SpokesPerScan() const { return numSpokes_; }
  double ScanPeriod() const { return period_; }
  double Dwell() const { return dwell_; }

  /// \brief Simulation time at which the beam axis points at spoke _g.
  double DueTime(int64_t _g) const;

  /// \brief Largest sweep index whose DueTime() <= _t (-1 if none).
  int64_t LastDueIndex(double _t) const;

  int64_t Revolution(int64_t _g) const;
  int SweepPosition(int64_t _g) const;

  /// \brief Azimuth index (0 = most clockwise / starboard edge of the FOV)
  /// of spoke _g. Azimuth increases with the index.
  int AzimuthIndex(int64_t _g) const;

  /// \brief REP-103 azimuth of spoke _g in the radar frame [rad].
  double SpokeAzimuth(int64_t _g) const;

  /// \brief Azimuth of the beam axis at time _t, wrapped to (-pi, pi].
  double AntennaAzimuth(double _t) const;

  /// \brief True if raw data of spoke _a may contribute to spoke _b through
  /// the beam pattern (always true for 360 degree scans; same revolution for
  /// sector scans, because the antenna is blanked between sectors).
  bool Contiguous(int64_t _a, int64_t _b) const;

 private:
  const RadarConfig cfg_;
  int numSpokes_;
  double spacing_;
  double period_;
  double dwell_;
};

class BeamModel
{
 public:
  explicit BeamModel(const RadarConfig &_cfg);

  /// \brief Unit direction in the radar frame for azimuth/elevation.
  static Vec3 Direction(double _azimuth, double _elevation);

  /// \brief Two-way horizontal gain [dB] at an angular offset from the beam
  /// axis: 0 dB on axis, -3 dB at +/- beamwidth/2.
  double HorizontalGainDb(double _offset) const;

  /// \brief Two-way vertical gain [dB] for an elevation angle.
  double VerticalGainDb(double _elevation) const;

  /// \brief Number of neighbouring spokes on each side whose raw returns
  /// fall inside the main lobe (|offset| <= beamwidth, i.e. >= -12 dB).
  int HalfBeamSpokes() const { return halfBeamSpokes_; }

 private:
  double bwH_;
  double bwV_;
  int halfBeamSpokes_;
};

}  // namespace marine_radar

#endif  // GZ_SENSORS_MARINE_RADAR_SCANGEOMETRY_HH_
