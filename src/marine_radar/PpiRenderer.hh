// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Component 5 of the pipeline: radar image generation.
//
// A Plan Position Indicator (PPI) is produced by scan-converting the polar
// echo video into a square RGB image: dark background, range rings, bearing
// lines, heading line, rotating sweep line with afterglow, echo video coloured
// by signal strength, markers for the targets of the last completed scan and
// a centre dot at the radar position.
//
// Display conventions (see docs/coordinate_frames.md):
//  * The radar is at the image centre. Image "up" is display angle 0 and
//    display angles grow counter-clockwise on screen.
//  * head_up:  up = radar +X (bow), left = radar +Y (port).
//  * north_up: up = world +Y (north in ENU), right = world +X (east).
//  * Pixel scale = EffectiveDisplayRange() / (image_size / 2 - 2) m/px.

#ifndef GZ_SENSORS_MARINE_RADAR_PPIRENDERER_HH_
#define GZ_SENSORS_MARINE_RADAR_PPIRENDERER_HH_

#include <cstdint>
#include <utility>
#include <vector>

#include "marine_radar/RadarConfig.hh"
#include "marine_radar/TargetExtraction.hh"
#include "marine_radar/Types.hh"

namespace marine_radar
{

/// \brief Polar video memory indexed by display angle and range bin.
class DisplayBuffer
{
 public:
  DisplayBuffer(int _slots, int _bins);

  int Slots() const { return slots_; }
  int Bins() const { return bins_; }

  /// \brief Slot index for a display angle [rad] (0 = up, CCW positive).
  int SlotForAngle(double _displayAngle) const;

  /// \brief Overwrite the slot at _displayAngle with new video.
  /// \param[in] _levels (bin, level) pairs; level 0 is "no echo".
  void Paint(double _displayAngle, double _time,
             const std::vector<std::pair<int, uint8_t>> &_levels);

  void Clear();

  uint8_t Level(int _slot, int _bin) const
  {
    return levels_[static_cast<size_t>(_slot) * static_cast<size_t>(bins_) +
                   static_cast<size_t>(_bin)];
  }

  double Time(int _slot) const { return time_[static_cast<size_t>(_slot)]; }

  /// \brief Non-zero (bin, level) entries of a slot.
  const std::vector<std::pair<int, uint8_t>> &SlotEchoes(int _slot) const
  {
    return sparse_[static_cast<size_t>(_slot)];
  }

 private:
  int slots_;
  int bins_;
  std::vector<uint8_t> levels_;
  std::vector<double> time_;
  std::vector<std::vector<std::pair<int, uint8_t>>> sparse_;
};

class PpiRenderer
{
 public:
  explicit PpiRenderer(const RadarConfig &_cfg);

  /// \brief Display angle of a radar-frame azimuth for the current mode.
  double DisplayAngle(double _radarAzimuth, double _radarYaw) const;

  /// \brief Pixel coordinates (may be fractional / off-image) of a point at
  /// horizontal range _range [m] and display angle _angle [rad].
  void ToPixel(double _range, double _angle, double &_u, double &_v) const;

  /// \brief Render the PPI.
  /// \param[in] _buffer polar video (indexed by display angle).
  /// \param[in] _time current simulation time [s].
  /// \param[in] _antennaAzimuth current beam azimuth in the radar frame.
  /// \param[in] _pose current radar pose in the world.
  /// \param[in] _targets targets of the last completed scan.
  void Render(const DisplayBuffer &_buffer, double _time,
              double _antennaAzimuth, const Pose &_pose,
              const std::vector<Target> &_targets, RgbImage &_out) const;

  double MetersPerPixel() const { return mPerPx_; }

 private:
  void DrawRadial(RgbImage &_img, double _angle, const uint8_t _rgb[3],
                  bool _thick) const;
  void DrawBox(RgbImage &_img, double _u, double _v, int _half,
               const uint8_t _rgb[3]) const;

  RadarConfig cfg_;
  int size_;
  double center_;
  double radiusPx_;
  double mPerPx_;
  double scanPeriod_;
  int bins_;
  std::vector<int32_t> cellLut_;   ///< slot * bins + bin, or -1.
  std::vector<uint8_t> overlay_;   ///< 0 none,1 inside,2 ring,3 bearing line
  std::vector<uint8_t> palette_;   ///< 256 * RGB
  std::vector<double> slotSin_;    ///< sin of slot centre display angles
  std::vector<double> slotCos_;
  std::vector<double> binPx_;      ///< bin centre range in pixels
  // Per-render scratch buffers (mutable: Render() is logically const).
  mutable std::vector<uint8_t> pixLevel_;
  mutable std::vector<float> pixAge_;
  mutable std::vector<uint8_t> expandLevel_;
  mutable std::vector<float> expandAge_;
};

}  // namespace marine_radar

#endif  // GZ_SENSORS_MARINE_RADAR_PPIRENDERER_HH_
