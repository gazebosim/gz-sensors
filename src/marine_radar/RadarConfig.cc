// Copyright 2026 marine_radar contributors. Apache-2.0.

#include "marine_radar/RadarConfig.hh"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>

namespace marine_radar
{
namespace
{

constexpr int kMaxSpokes = 16384;
constexpr int kMaxBins = 16384;
constexpr double kMaxCells = 64.0e6;

std::string Trim(const std::string &_s)
{
  size_t b = 0;
  size_t e = _s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(_s[b])))
    ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(_s[e - 1])))
    --e;
  return _s.substr(b, e - b);
}

std::string Fmt(double _v)
{
  std::ostringstream os;
  os << std::setprecision(8) << _v;
  return os.str();
}

bool ParseDouble(const std::string &_text, double &_out)
{
  const std::string t = Trim(_text);
  if (t.empty())
    return false;
  char *end = nullptr;
  errno = 0;
  const double v = std::strtod(t.c_str(), &end);
  if (end == t.c_str() || *end != '\0' || errno == ERANGE || !std::isfinite(v))
    return false;
  _out = v;
  return true;
}

bool ParseInt(const std::string &_text, int64_t &_out)
{
  const std::string t = Trim(_text);
  if (t.empty())
    return false;
  char *end = nullptr;
  errno = 0;
  const int64_t v = static_cast<int64_t>(std::strtoll(t.c_str(), &end, 10));
  if (end == t.c_str() || *end != '\0' || errno == ERANGE)
    return false;
  _out = v;
  return true;
}

bool ParseBool(const std::string &_text, bool &_out)
{
  std::string t = Trim(_text);
  std::transform(t.begin(), t.end(), t.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (t == "true" || t == "1" || t == "yes" || t == "on")
  {
    _out = true;
    return true;
  }
  if (t == "false" || t == "0" || t == "no" || t == "off")
  {
    _out = false;
    return true;
  }
  return false;
}

void Check(bool _ok, std::vector<std::string> &_errors,
           const std::string &_name, double _value, const std::string &_rule)
{
  if (!_ok)
  {
    _errors.push_back("parameter <" + _name + "> = " + Fmt(_value) +
                      " is invalid: " + _rule);
  }
}

}  // namespace

/////////////////////////////////////////////////
const std::vector<std::string> &KnownConfigKeys()
{
  static const std::vector<std::string> keys = {
    "range_min", "range_max", "range_resolution",
    "horizontal_fov_deg", "angular_resolution_deg", "samples",
    "rotation_rate_rpm",
    "beamwidth_horizontal_deg", "beamwidth_vertical_deg", "slice_heights",
    "snr_reference_db", "reference_range", "detection_threshold_db",
    "intensity_noise_stddev_db", "range_noise_stddev",
    "bearing_noise_stddev_deg", "echo_detection_probability",
    "false_alarm_rate", "sea_level_filter", "sea_level",
    "min_target_cells", "cluster_spoke_gap", "cluster_bin_gap",
    "association_gate", "track_coast_scans",
    "image_size", "image_rate", "display_range", "range_rings",
    "bearing_line_spacing_deg", "display_mode", "afterglow",
    "draw_target_markers", "echo_expansion", "display_dynamic_range_db",
    "seed"};
  return keys;
}

/////////////////////////////////////////////////
std::vector<std::string> RadarConfig::Validate() const
{
  std::vector<std::string> e;

  Check(std::isfinite(range_min) && range_min >= 0.0, e, "range_min",
        range_min, "must be >= 0 m");
  Check(std::isfinite(range_max) && range_max > range_min, e, "range_max",
        range_max, "must be greater than range_min (" + Fmt(range_min) + " m)");
  Check(range_max <= 200000.0, e, "range_max", range_max,
        "must be <= 200000 m");
  Check(std::isfinite(range_resolution) && range_resolution > 0.0, e,
        "range_resolution", range_resolution, "must be > 0 m");
  if (e.empty())
  {
    Check(range_resolution <= range_max - range_min, e, "range_resolution",
          range_resolution, "must not exceed range_max - range_min");
    const double bins = std::ceil((range_max - range_min) / range_resolution);
    Check(bins <= kMaxBins, e, "range_resolution", range_resolution,
          "gives " + Fmt(bins) + " range bins (max " +
          std::to_string(kMaxBins) +
          "); increase range_resolution for long ranges");
  }

  Check(std::isfinite(horizontal_fov) && horizontal_fov > 0.0 &&
        horizontal_fov <= kTwoPi + 1e-9, e, "horizontal_fov_deg",
        RadToDeg(horizontal_fov), "must be in (0, 360] degrees");
  Check(std::isfinite(angular_resolution) && angular_resolution > 0.0 &&
        angular_resolution <= horizontal_fov + 1e-12, e,
        "angular_resolution_deg", RadToDeg(angular_resolution),
        "must be > 0 and <= horizontal_fov_deg");
  if (std::isfinite(angular_resolution) && angular_resolution > 0.0 &&
      std::isfinite(horizontal_fov) && horizontal_fov > 0.0)
  {
    const double spokes = std::round(horizontal_fov / angular_resolution);
    Check(spokes <= kMaxSpokes, e, "angular_resolution_deg",
          RadToDeg(angular_resolution),
          "gives " + Fmt(spokes) + " spokes per scan (max " +
          std::to_string(kMaxSpokes) + ")");
    if (range_resolution > 0.0 && range_max > range_min)
    {
      const double cells = spokes *
          std::ceil((range_max - range_min) / range_resolution);
      Check(cells <= kMaxCells, e, "angular_resolution_deg",
            RadToDeg(angular_resolution),
            "spokes x range bins = " + Fmt(cells) +
            " polar cells exceeds the limit of 64e6; coarsen the angular "
            "or range resolution");
    }
  }

  Check(std::isfinite(rotation_rate_rpm) && rotation_rate_rpm > 0.0 &&
        rotation_rate_rpm <= 600.0, e, "rotation_rate_rpm",
        rotation_rate_rpm, "must be in (0, 600] rpm");

  Check(std::isfinite(beamwidth_horizontal) && beamwidth_horizontal > 0.0 &&
        beamwidth_horizontal <= DegToRad(45.0), e, "beamwidth_horizontal_deg",
        RadToDeg(beamwidth_horizontal), "must be in (0, 45] degrees");
  Check(std::isfinite(beamwidth_vertical) && beamwidth_vertical > 0.0 &&
        beamwidth_vertical <= DegToRad(90.0), e, "beamwidth_vertical_deg",
        RadToDeg(beamwidth_vertical), "must be in (0, 90] degrees");
  Check(!slice_heights.empty() && slice_heights.size() <= 64, e,
        "slice_heights", static_cast<double>(slice_heights.size()),
        "must list between 1 and 64 heights (number of entries shown)");
  for (double h : slice_heights)
  {
    Check(std::isfinite(h) && h >= 0.0 && h <= 1000.0, e, "slice_heights",
          h, "every height must be in [0, 1000] m above sea_level");
  }

  Check(std::isfinite(snr_reference_db), e, "snr_reference_db",
        snr_reference_db, "must be finite");
  Check(std::isfinite(reference_range) && reference_range > 0.0, e,
        "reference_range", reference_range, "must be > 0 m");
  Check(std::isfinite(detection_threshold_db), e, "detection_threshold_db",
        detection_threshold_db, "must be finite");
  Check(std::isfinite(intensity_noise_stddev_db) &&
        intensity_noise_stddev_db >= 0.0, e, "intensity_noise_stddev_db",
        intensity_noise_stddev_db, "must be >= 0");
  Check(std::isfinite(range_noise_stddev) && range_noise_stddev >= 0.0, e,
        "range_noise_stddev", range_noise_stddev, "must be >= 0 m");
  Check(std::isfinite(bearing_noise_stddev) && bearing_noise_stddev >= 0.0,
        e, "bearing_noise_stddev_deg", RadToDeg(bearing_noise_stddev),
        "must be >= 0 degrees");
  Check(std::isfinite(echo_detection_probability) &&
        echo_detection_probability >= 0.0 &&
        echo_detection_probability <= 1.0, e, "echo_detection_probability",
        echo_detection_probability, "must be in [0, 1]");
  Check(std::isfinite(false_alarm_rate) && false_alarm_rate >= 0.0 &&
        false_alarm_rate <= 10000.0, e, "false_alarm_rate", false_alarm_rate,
        "must be in [0, 10000] false echoes per scan");
  Check(std::isfinite(sea_level), e, "sea_level", sea_level,
        "must be finite");

  Check(min_target_cells >= 1, e, "min_target_cells", min_target_cells,
        "must be >= 1");
  Check(cluster_spoke_gap >= 0 && cluster_spoke_gap <= 16, e,
        "cluster_spoke_gap", cluster_spoke_gap, "must be in [0, 16]");
  Check(cluster_bin_gap >= 0 && cluster_bin_gap <= 64, e, "cluster_bin_gap",
        cluster_bin_gap, "must be in [0, 64]");
  Check(std::isfinite(association_gate) && association_gate >= 0.0, e,
        "association_gate", association_gate, "must be >= 0 m");
  Check(track_coast_scans >= 0 && track_coast_scans <= 100, e,
        "track_coast_scans", track_coast_scans, "must be in [0, 100]");

  Check(image_size >= 64 && image_size <= 4096, e, "image_size", image_size,
        "must be in [64, 4096] pixels");
  Check(std::isfinite(image_rate) && image_rate >= 0.0 && image_rate <= 100.0,
        e, "image_rate", image_rate, "must be in [0, 100] Hz (0 disables)");
  Check(std::isfinite(display_range) && display_range >= 0.0, e,
        "display_range", display_range,
        "must be >= 0 m (0 means: follow range_max)");
  Check(range_rings >= 0 && range_rings <= 50, e, "range_rings", range_rings,
        "must be in [0, 50]");
  Check(std::isfinite(bearing_line_spacing) && bearing_line_spacing >= 0.0 &&
        bearing_line_spacing <= kTwoPi, e, "bearing_line_spacing_deg",
        RadToDeg(bearing_line_spacing), "must be in [0, 360] degrees");
  Check(std::isfinite(afterglow) && afterglow >= 0.0 && afterglow <= 1.0, e,
        "afterglow", afterglow, "must be in [0, 1]");
  Check(std::isfinite(display_dynamic_range_db) &&
        display_dynamic_range_db >= 1.0 && display_dynamic_range_db <= 150.0,
        e, "display_dynamic_range_db", display_dynamic_range_db,
        "must be in [1, 150] dB");
  return e;
}

/////////////////////////////////////////////////
bool RadarConfig::FullCircle() const
{
  return horizontal_fov >= kTwoPi - 1e-9;
}

/////////////////////////////////////////////////
int RadarConfig::NumSpokes() const
{
  const double n = std::round(horizontal_fov / angular_resolution);
  return std::max(1, static_cast<int>(n));
}

/////////////////////////////////////////////////
int RadarConfig::NumBins() const
{
  const double n = std::ceil((range_max - range_min) / range_resolution - 1e-9);
  return std::max(1, static_cast<int>(n));
}

/////////////////////////////////////////////////
double RadarConfig::SpokeSpacing() const
{
  return horizontal_fov / static_cast<double>(NumSpokes());
}

/////////////////////////////////////////////////
double RadarConfig::ScanPeriod() const
{
  return 60.0 / rotation_rate_rpm;
}

/////////////////////////////////////////////////
double RadarConfig::SpokeDwell() const
{
  return ScanPeriod() * SpokeSpacing() / kTwoPi;
}

/////////////////////////////////////////////////
double RadarConfig::SpokeAzimuth(int _i) const
{
  return WrapAngle(-horizontal_fov / 2.0 +
                   (static_cast<double>(_i) + 0.5) * SpokeSpacing());
}

/////////////////////////////////////////////////
double RadarConfig::EffectiveDisplayRange() const
{
  return display_range > 0.0 ? display_range : range_max;
}

/////////////////////////////////////////////////
double RadarConfig::BinCenterRange(int _bin) const
{
  return range_min + (static_cast<double>(_bin) + 0.5) * range_resolution;
}

/////////////////////////////////////////////////
ParseResult ParseConfig(const std::map<std::string, std::string> &_params)
{
  ParseResult res;
  RadarConfig &c = res.config;
  const auto &known = KnownConfigKeys();

  for (const auto &kv : _params)
  {
    if (std::find(known.begin(), known.end(), kv.first) == known.end())
    {
      res.warnings.push_back("unknown parameter <" + kv.first +
                             "> ignored (check spelling)");
    }
  }

  auto getDouble = [&](const std::string &_key, double &_dst,
                       double _scale = 1.0) -> bool
  {
    auto it = _params.find(_key);
    if (it == _params.end())
      return false;
    double v = 0.0;
    if (!ParseDouble(it->second, v))
    {
      res.errors.push_back("parameter <" + _key + "> = '" + it->second +
                           "' is not a valid number");
      return false;
    }
    _dst = v * _scale;
    return true;
  };
  auto getInt = [&](const std::string &_key, int &_dst) -> bool
  {
    auto it = _params.find(_key);
    if (it == _params.end())
      return false;
    int64_t v = 0;
    if (!ParseInt(it->second, v) || v < std::numeric_limits<int>::min() ||
        v > std::numeric_limits<int>::max())
    {
      res.errors.push_back("parameter <" + _key + "> = '" + it->second +
                           "' is not a valid integer");
      return false;
    }
    _dst = static_cast<int>(v);
    return true;
  };
  auto getBool = [&](const std::string &_key, bool &_dst) -> bool
  {
    auto it = _params.find(_key);
    if (it == _params.end())
      return false;
    if (!ParseBool(it->second, _dst))
    {
      res.errors.push_back("parameter <" + _key + "> = '" + it->second +
                           "' is not a valid boolean (use true/false)");
      return false;
    }
    return true;
  };

  const double d2r = kPi / 180.0;
  getDouble("range_min", c.range_min);
  getDouble("range_max", c.range_max);
  getDouble("range_resolution", c.range_resolution);
  getDouble("horizontal_fov_deg", c.horizontal_fov, d2r);
  const bool hasRes =
      getDouble("angular_resolution_deg", c.angular_resolution, d2r);
  int samples = 0;
  const bool hasSamples = getInt("samples", samples);
  getDouble("rotation_rate_rpm", c.rotation_rate_rpm);
  getDouble("beamwidth_horizontal_deg", c.beamwidth_horizontal, d2r);
  getDouble("beamwidth_vertical_deg", c.beamwidth_vertical, d2r);
  {
    auto it = _params.find("slice_heights");
    if (it != _params.end())
    {
      std::string text = it->second;
      std::replace(text.begin(), text.end(), ',', ' ');
      std::istringstream is(text);
      std::vector<double> heights;
      std::string tok;
      bool okList = true;
      while (is >> tok)
      {
        double v = 0.0;
        if (!ParseDouble(tok, v))
        {
          okList = false;
          break;
        }
        heights.push_back(v);
      }
      if (!okList || heights.empty())
      {
        res.errors.push_back("parameter <slice_heights> = '" + it->second +
                             "' must be a space separated list of numbers");
      }
      else
      {
        c.slice_heights = heights;
      }
    }
  }
  getDouble("snr_reference_db", c.snr_reference_db);
  getDouble("reference_range", c.reference_range);
  getDouble("detection_threshold_db", c.detection_threshold_db);
  getDouble("intensity_noise_stddev_db", c.intensity_noise_stddev_db);
  getDouble("range_noise_stddev", c.range_noise_stddev);
  getDouble("bearing_noise_stddev_deg", c.bearing_noise_stddev, d2r);
  getDouble("echo_detection_probability", c.echo_detection_probability);
  getDouble("false_alarm_rate", c.false_alarm_rate);
  getBool("sea_level_filter", c.sea_level_filter);
  getDouble("sea_level", c.sea_level);
  getInt("min_target_cells", c.min_target_cells);
  getInt("cluster_spoke_gap", c.cluster_spoke_gap);
  getInt("cluster_bin_gap", c.cluster_bin_gap);
  getDouble("association_gate", c.association_gate);
  getInt("track_coast_scans", c.track_coast_scans);
  getInt("image_size", c.image_size);
  getDouble("image_rate", c.image_rate);
  getDouble("display_range", c.display_range);
  getInt("range_rings", c.range_rings);
  getDouble("bearing_line_spacing_deg", c.bearing_line_spacing, d2r);
  getDouble("afterglow", c.afterglow);
  getBool("draw_target_markers", c.draw_target_markers);
  getBool("echo_expansion", c.echo_expansion);
  getDouble("display_dynamic_range_db", c.display_dynamic_range_db);
  {
    int64_t seed = 0;
    auto it = _params.find("seed");
    if (it != _params.end())
    {
      if (!ParseInt(it->second, seed) || seed < 0 || seed > 0xFFFFFFFFLL)
        res.errors.push_back("parameter <seed> = '" + it->second +
                             "' must be an integer in [0, 4294967295]");
      else
        c.seed = static_cast<uint32_t>(seed);
    }
  }
  {
    auto it = _params.find("display_mode");
    if (it != _params.end())
    {
      const std::string m = Trim(it->second);
      if (m == "head_up")
        c.north_up = false;
      else if (m == "north_up")
        c.north_up = true;
      else
        res.errors.push_back("parameter <display_mode> = '" + it->second +
                             "' must be 'head_up' or 'north_up'");
    }
  }

  if (hasSamples)
  {
    if (samples < 1 || samples > kMaxSpokes)
    {
      res.errors.push_back("parameter <samples> = " + std::to_string(samples) +
                           " must be in [1, " + std::to_string(kMaxSpokes) +
                           "]");
    }
    else
    {
      const double fromSamples = c.horizontal_fov / samples;
      if (hasRes && std::abs(fromSamples - c.angular_resolution) > 1e-6)
      {
        res.errors.push_back(
            "parameters <samples> = " + std::to_string(samples) +
            " and <angular_resolution_deg> = " +
            Fmt(RadToDeg(c.angular_resolution)) +
            " disagree for horizontal_fov_deg = " +
            Fmt(RadToDeg(c.horizontal_fov)) + " (fov / samples = " +
            Fmt(RadToDeg(fromSamples)) + " deg); specify only one of them");
      }
      c.angular_resolution = fromSamples;
    }
  }

  if (!res.errors.empty())
    return res;

  const auto validation = c.Validate();
  res.errors.insert(res.errors.end(), validation.begin(), validation.end());

  if (res.errors.empty())
  {
    const double exact = c.SpokeSpacing();
    if (std::abs(exact - c.angular_resolution) > 1e-9)
    {
      res.warnings.push_back(
          "angular_resolution_deg adjusted from " +
          Fmt(RadToDeg(c.angular_resolution)) + " to " +
          Fmt(RadToDeg(exact)) + " so the field of view holds exactly " +
          std::to_string(c.NumSpokes()) + " spokes");
      c.angular_resolution = exact;
    }
    if (c.beamwidth_horizontal < c.angular_resolution)
    {
      res.warnings.push_back(
          "beamwidth_horizontal_deg is smaller than the spoke spacing; "
          "small targets may fall between spokes and be missed");
    }
  }
  return res;
}

/////////////////////////////////////////////////
std::string Summary(const RadarConfig &_c)
{
  std::ostringstream os;
  os << std::setprecision(6)
     << "range [" << _c.range_min << ", " << _c.range_max << "] m, "
     << _c.NumBins() << " bins of " << _c.range_resolution << " m; "
     << "fov " << RadToDeg(_c.horizontal_fov) << " deg, "
     << _c.NumSpokes() << " spokes (" << RadToDeg(_c.SpokeSpacing())
     << " deg); " << _c.rotation_rate_rpm << " rpm (scan period "
     << _c.ScanPeriod() << " s); beam " << RadToDeg(_c.beamwidth_horizontal)
     << " x " << RadToDeg(_c.beamwidth_vertical) << " deg with "
     << _c.slice_heights.size() << " height slices; image " << _c.image_size
     << "px @ "
     << _c.image_rate << " Hz (" << (_c.north_up ? "north-up" : "head-up")
     << ")";
  return os.str();
}

}  // namespace marine_radar
