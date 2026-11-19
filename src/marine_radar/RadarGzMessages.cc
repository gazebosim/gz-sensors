// Copyright 2026 marine_radar contributors. Apache-2.0.

#include "RadarGzMessages.hh"

#include <gz/msgs/PointCloudPackedUtils.hh>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

namespace marine_radar
{
namespace
{
using Field = gz::msgs::PointCloudPacked::Field;

/// \brief Sequential writer for one packed point.
class PointWriter
{
 public:
  explicit PointWriter(char *_dst) : dst_(_dst) {}
  void F64(double _v) { Put(&_v, sizeof(_v)); }
  void F32(double _v)
  {
    const float f = static_cast<float>(_v);
    Put(&f, sizeof(f));
  }
  void U32(uint32_t _v) { Put(&_v, sizeof(_v)); }

 private:
  void Put(const void *_src, size_t _n)
  {
    std::memcpy(dst_ + off_, _src, _n);
    off_ += _n;
  }
  char *dst_;
  size_t off_{0};
};

void InitCloud(gz::msgs::PointCloudPacked &_msg, const std::string &_frameId,
               const std::vector<std::pair<std::string, Field::DataType>> &_f,
               uint32_t _numPoints, double _stamp)
{
  _msg.Clear();
  gz::msgs::InitPointCloudPacked(_msg, _frameId, false, _f);
  FillHeader(_stamp, _frameId, *_msg.mutable_header());
  _msg.set_height(1);
  _msg.set_width(_numPoints);
  _msg.set_is_bigendian(false);
  _msg.set_is_dense(true);
  _msg.set_row_step(_msg.point_step() * _numPoints);
  _msg.mutable_data()->resize(
      static_cast<size_t>(_msg.point_step()) * _numPoints);
}
}  // namespace

/////////////////////////////////////////////////
void FillHeader(double _stamp, const std::string &_frameId,
                gz::msgs::Header &_header)
{
  double sec = std::floor(_stamp);
  int64_t nsec = static_cast<int64_t>(std::llround((_stamp - sec) * 1e9));
  if (nsec >= 1000000000)
  {
    sec += 1.0;
    nsec -= 1000000000;
  }
  _header.mutable_stamp()->set_sec(static_cast<int64_t>(sec));
  _header.mutable_stamp()->set_nsec(static_cast<int32_t>(nsec));
  _header.clear_data();
  auto *frame = _header.add_data();
  frame->set_key("frame_id");
  frame->add_value(_frameId);
}

/////////////////////////////////////////////////
void FillImage(const marine_radar::RgbImage &_img, double _stamp,
               const std::string &_frameId, gz::msgs::Image &_msg)
{
  FillHeader(_stamp, _frameId, *_msg.mutable_header());
  _msg.set_width(static_cast<uint32_t>(_img.width));
  _msg.set_height(static_cast<uint32_t>(_img.height));
  _msg.set_step(static_cast<uint32_t>(_img.width * 3));
  _msg.set_pixel_format_type(gz::msgs::PixelFormatType::RGB_INT8);
  _msg.set_data(reinterpret_cast<const char *>(_img.data.data()),
                _img.data.size());
}

/////////////////////////////////////////////////
void FillTargets(const marine_radar::ScanResult &_scan,
                 const std::string &_frameId,
                 gz::msgs::PointCloudPacked &_msg)
{
  static const std::vector<std::pair<std::string, Field::DataType>> fields = {
    {"world_x", Field::FLOAT64}, {"world_y", Field::FLOAT64},
    {"world_z", Field::FLOAT64},
    {"x", Field::FLOAT32}, {"y", Field::FLOAT32}, {"z", Field::FLOAT32},
    {"range", Field::FLOAT32}, {"bearing", Field::FLOAT32},
    {"elevation", Field::FLOAT32}, {"intensity", Field::FLOAT32},
    {"id", Field::UINT32}, {"age", Field::UINT32},
    {"num_cells", Field::UINT32},
    {"min_range", Field::FLOAT32}, {"range_extent", Field::FLOAT32},
    {"bearing_extent", Field::FLOAT32}, {"time_offset", Field::FLOAT32}};

  const uint32_t n = static_cast<uint32_t>(_scan.targets.size());
  InitCloud(_msg, _frameId, fields, n, _scan.stamp);
  if (n == 0)
    return;
  char *data = &(*_msg.mutable_data())[0];
  for (uint32_t i = 0; i < n; ++i)
  {
    const auto &t = _scan.targets[i];
    PointWriter w(data + static_cast<size_t>(i) * _msg.point_step());
    w.F64(t.position_world.x);
    w.F64(t.position_world.y);
    w.F64(t.position_world.z);
    w.F32(t.position_radar.x);
    w.F32(t.position_radar.y);
    w.F32(t.position_radar.z);
    w.F32(t.range);
    w.F32(t.bearing);
    w.F32(t.elevation);
    w.F32(t.intensity_db);
    w.U32(t.id);
    w.U32(t.age);
    w.U32(t.num_cells);
    w.F32(t.min_range);
    w.F32(t.range_extent);
    w.F32(t.bearing_extent);
    w.F32(t.time - _scan.stamp);
  }
}

/////////////////////////////////////////////////
void FillReturns(const marine_radar::ScanResult &_scan,
                 const std::string &_frameId,
                 gz::msgs::PointCloudPacked &_msg)
{
  static const std::vector<std::pair<std::string, Field::DataType>> fields = {
    {"world_x", Field::FLOAT64}, {"world_y", Field::FLOAT64},
    {"world_z", Field::FLOAT64},
    {"x", Field::FLOAT32}, {"y", Field::FLOAT32}, {"z", Field::FLOAT32},
    {"intensity", Field::FLOAT32}, {"range", Field::FLOAT32},
    {"bearing", Field::FLOAT32}, {"elevation", Field::FLOAT32},
    {"time_offset", Field::FLOAT32}};

  const uint32_t n = static_cast<uint32_t>(_scan.returns.size());
  InitCloud(_msg, _frameId, fields, n, _scan.stamp);
  if (n == 0)
    return;
  char *data = &(*_msg.mutable_data())[0];
  for (uint32_t i = 0; i < n; ++i)
  {
    const auto &r = _scan.returns[i];
    PointWriter w(data + static_cast<size_t>(i) * _msg.point_step());
    w.F64(r.position_world.x);
    w.F64(r.position_world.y);
    w.F64(r.position_world.z);
    w.F32(r.position_radar.x);
    w.F32(r.position_radar.y);
    w.F32(r.position_radar.z);
    w.F32(r.intensity_db);
    w.F32(r.range);
    w.F32(r.bearing);
    w.F32(r.elevation);
    w.F32(r.time - _scan.stamp);
  }
}

}  // namespace marine_radar
