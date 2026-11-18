// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Minimal geometry types used by the radar core library. The core library is
// deliberately free of Gazebo and ROS dependencies so it can be unit tested
// and reused with any geometry back end.

#ifndef GZ_SENSORS_MARINE_RADAR_TYPES_HH_
#define GZ_SENSORS_MARINE_RADAR_TYPES_HH_

#include <cmath>
#include <cstdint>
#include <vector>

namespace marine_radar
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

inline double DegToRad(double _deg) { return _deg * kPi / 180.0; }
inline double RadToDeg(double _rad) { return _rad * 180.0 / kPi; }

/// \brief Wrap an angle to the interval (-pi, pi].
inline double WrapAngle(double _a)
{
  double r = std::fmod(_a + kPi, kTwoPi);
  if (r < 0.0)
    r += kTwoPi;
  r -= kPi;
  // fmod maps +pi to -pi; keep +pi so the interval is (-pi, pi].
  if (r <= -kPi)
    r = kPi;
  return r;
}

/// \brief Wrap an angle to the interval [0, 2*pi).
inline double WrapAngle2Pi(double _a)
{
  double r = std::fmod(_a, kTwoPi);
  if (r < 0.0)
    r += kTwoPi;
  if (r >= kTwoPi)
    r -= kTwoPi;
  return r;
}

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Vec3() = default;
  Vec3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

  Vec3 operator+(const Vec3 &_o) const
  {
    return {x + _o.x, y + _o.y, z + _o.z};
  }
  Vec3 operator-(const Vec3 &_o) const
  {
    return {x - _o.x, y - _o.y, z - _o.z};
  }
  Vec3 operator*(double _s) const { return {x * _s, y * _s, z * _s}; }
  double Dot(const Vec3 &_o) const { return x * _o.x + y * _o.y + z * _o.z; }
  double Length() const { return std::sqrt(x * x + y * y + z * z); }
  bool IsFinite() const
  {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }
};

/// \brief Unit quaternion (w, x, y, z).
struct Quat
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Quat() = default;
  Quat(double _w, double _x, double _y, double _z)
    : w(_w), x(_x), y(_y), z(_z) {}

  static Quat FromYaw(double _yaw)
  {
    return {std::cos(_yaw / 2.0), 0.0, 0.0, std::sin(_yaw / 2.0)};
  }

  static Quat FromRPY(double _r, double _p, double _y)
  {
    const double cr = std::cos(_r / 2), sr = std::sin(_r / 2);
    const double cp = std::cos(_p / 2), sp = std::sin(_p / 2);
    const double cy = std::cos(_y / 2), sy = std::sin(_y / 2);
    return {cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy};
  }

  Quat Conjugate() const { return {w, -x, -y, -z}; }

  Quat operator*(const Quat &_q) const
  {
    return {w * _q.w - x * _q.x - y * _q.y - z * _q.z,
            w * _q.x + x * _q.w + y * _q.z - z * _q.y,
            w * _q.y - x * _q.z + y * _q.w + z * _q.x,
            w * _q.z + x * _q.y - y * _q.x + z * _q.w};
  }

  Vec3 Rotate(const Vec3 &_v) const
  {
    // v' = q * v * q^-1, expanded.
    const double tx = 2.0 * (y * _v.z - z * _v.y);
    const double ty = 2.0 * (z * _v.x - x * _v.z);
    const double tz = 2.0 * (x * _v.y - y * _v.x);
    return {_v.x + w * tx + (y * tz - z * ty),
            _v.y + w * ty + (z * tx - x * tz),
            _v.z + w * tz + (x * ty - y * tx)};
  }

  Vec3 RotateReverse(const Vec3 &_v) const { return Conjugate().Rotate(_v); }

  /// \brief Yaw angle (rotation about +Z) of this orientation.
  double Yaw() const
  {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }
};

/// \brief Rigid transform. Transform(p) maps a point from the local frame
/// into the parent frame.
struct Pose
{
  Vec3 position;
  Quat orientation;

  Vec3 Transform(const Vec3 &_local) const
  {
    return position + orientation.Rotate(_local);
  }

  Vec3 InverseTransform(const Vec3 &_parent) const
  {
    return orientation.RotateReverse(_parent - position);
  }

  double Yaw() const { return orientation.Yaw(); }
};

/// \brief 8-bit RGB image, row-major, origin at the top-left pixel.
struct RgbImage
{
  int width{0};
  int height{0};
  std::vector<uint8_t> data;  // width * height * 3

  void Resize(int _w, int _h)
  {
    width = _w;
    height = _h;
    data.assign(static_cast<size_t>(_w) * static_cast<size_t>(_h) * 3u, 0u);
  }

  const uint8_t *Pixel(int _u, int _v) const
  {
    return &data[(static_cast<size_t>(_v) * static_cast<size_t>(width) +
                  static_cast<size_t>(_u)) * 3u];
  }

  uint8_t *Pixel(int _u, int _v)
  {
    return &data[(static_cast<size_t>(_v) * static_cast<size_t>(width) +
                  static_cast<size_t>(_u)) * 3u];
  }
};

}  // namespace marine_radar

#endif  // GZ_SENSORS_MARINE_RADAR_TYPES_HH_
