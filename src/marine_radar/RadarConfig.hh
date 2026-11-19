// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Component 1 of the radar pipeline: configuration, defaults, validation and
// derived quantities. Parsing works from a plain key/value map so it can be
// unit tested without SDFormat; the Gazebo plugin converts its <plugin> SDF
// element into such a map.

#ifndef GZ_SENSORS_MARINE_RADAR_RADARCONFIG_HH_
#define GZ_SENSORS_MARINE_RADAR_RADARCONFIG_HH_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "marine_radar/Types.hh"

namespace marine_radar
{

/// \brief All radar parameters. Angles are stored in radians, distances in
/// metres, times in seconds. The SDF/key-value interface uses degrees for
/// angles (keys ending in "_deg").
struct RadarConfig
{
  // ---- Range ---------------------------------------------------------------
  double range_min{10.0};          ///< Minimum (blind) range [m].
  double range_max{1000.0};        ///< Maximum instrumented range [m].
  double range_resolution{2.0};    ///< Range-bin size (pulse length cell) [m].

  // ---- Antenna scan --------------------------------------------------------
  double horizontal_fov{kTwoPi};   ///< Transmitting sector, centred on +X.
  /// Spoke spacing [rad]. Default: 4096 spokes per revolution, typical of
  /// solid-state marine radars. Objects narrower than
  /// range * angular_resolution can fall between rays.
  double angular_resolution{kTwoPi / 4096.0};
  double rotation_rate_rpm{24.0};  ///< Antenna speed; one scan per revolution.

  // ---- Beam ----------------------------------------------------------------
  double beamwidth_horizontal{DegToRad(1.2)};  ///< Two-way -3 dB width.
  double beamwidth_vertical{DegToRad(20.0)};   ///< Two-way -3 dB width.
  /// Heights above sea_level [m] of the gravity-levelled ray slices cast
  /// along every spoke. Each slice is one ray per spoke.
  std::vector<double> slice_heights{0.5, 1.5, 3.0, 6.0, 12.0, 24.0};

  // ---- Echo / detection model ----------------------------------------------
  double snr_reference_db{30.0};   ///< SNR of a normal-incidence hit at
                                   ///< reference_range on the beam axis.
  double reference_range{1000.0};  ///< [m]
  double detection_threshold_db{8.0};  ///< Echoes below this SNR are dropped.
  double intensity_noise_stddev_db{1.5};  ///< Per-echo SNR fluctuation.
  double range_noise_stddev{0.0};  ///< Per-echo range noise [m].
  double bearing_noise_stddev{0.0};  ///< Per-target bearing noise [rad].
  double echo_detection_probability{1.0};  ///< Probability each echo
                                           ///< segment is received.
  double false_alarm_rate{0.0};    ///< Expected false echoes per scan.
  bool sea_level_filter{true};     ///< Ignore hits below sea_level (world z).
  double sea_level{0.0};           ///< [m] world z of the sea surface.

  // ---- Target extraction / identification ----------------------------------
  int min_target_cells{1};         ///< Clusters smaller than this are dropped.
  int cluster_spoke_gap{1};        ///< Missing spokes bridged when clustering.
  int cluster_bin_gap{2};          ///< Missing range bins bridged when
                                   ///< clustering.
  double association_gate{25.0};   ///< [m] max world distance for ID reuse.
  int track_coast_scans{2};        ///< Scans an ID survives without a match.

  // ---- Display (PPI image) -------------------------------------------------
  int image_size{512};             ///< Square image width = height [px].
  double image_rate{5.0};          ///< [Hz] 0 disables the image.
  double display_range{0.0};       ///< [m] 0 => follow range_max.
  int range_rings{4};
  double bearing_line_spacing{DegToRad(30.0)};  ///< 0 disables the lines.
  bool north_up{false};            ///< false => head-up display.
  double afterglow{0.6};           ///< Brightness lost over one revolution.
  bool draw_target_markers{true};
  bool echo_expansion{true};       ///< Grow echoes by one pixel on the PPI.
  double display_dynamic_range_db{50.0};  ///< SNR span (above the detection
                                          ///< threshold) of the colour scale.

  // ---- Misc ----------------------------------------------------------------
  uint32_t seed{42};               ///< Random seed (deterministic by default).

  /// \brief Check every parameter. Returns one human-readable message per
  /// problem; an empty vector means the configuration is valid.
  std::vector<std::string> Validate() const;

  // ---- Derived quantities (valid only for a validated config) ---------------
  bool FullCircle() const;
  int NumSpokes() const;            ///< Spokes per scan inside the FOV.
  int NumBins() const;              ///< Range bins per spoke.
  double SpokeSpacing() const;      ///< Exact spacing = fov / NumSpokes().
  double ScanPeriod() const;        ///< [s] one revolution.
  double SpokeDwell() const;        ///< [s] time between adjacent spokes.
  /// Azimuth (REP-103, CCW from +X) of azimuth index _i in [0, NumSpokes()).
  double SpokeAzimuth(int _i) const;
  double EffectiveDisplayRange() const;
  double BinCenterRange(int _bin) const;
};

/// \brief Result of parsing a key/value parameter map.
struct ParseResult
{
  RadarConfig config;
  std::vector<std::string> errors;    ///< Fatal: the radar must not start.
  std::vector<std::string> warnings;  ///< Non-fatal (e.g. unknown keys).
  bool Ok() const { return errors.empty(); }
};

/// \brief Build a configuration from string key/values (e.g. SDF children).
/// Unknown keys produce warnings, malformed numbers and invalid values
/// produce errors. Validate() is run on the result.
ParseResult ParseConfig(const std::map<std::string, std::string> &_params);

/// \brief List of all keys understood by ParseConfig (for docs and checks).
const std::vector<std::string> &KnownConfigKeys();

/// \brief Human-readable one-line summary of the main parameters.
std::string Summary(const RadarConfig &_cfg);

}  // namespace marine_radar

#endif  // GZ_SENSORS_MARINE_RADAR_RADARCONFIG_HH_
