// Copyright 2026 marine_radar contributors. Apache-2.0.
//
// Component 6 of the pipeline: detection / image message generation for
// Gazebo Transport. Kept separate from the system plugin so the message
// layouts can be unit tested.
//
// Targets and returns are published as gz.msgs.PointCloudPacked because
// ros_gz_bridge converts that type to sensor_msgs/msg/PointCloud2 without
// losing any of the named per-target fields. Field layouts are documented in
// docs/interfaces.md and are naturally aligned (FLOAT64 fields first, point
// size a multiple of 8 bytes).

#ifndef GZ_SENSORS_MARINE_RADAR_RADARGZMESSAGES_HH_
#define GZ_SENSORS_MARINE_RADAR_RADARGZMESSAGES_HH_

#include <gz/msgs/header.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>

#include <string>

#include "marine_radar/RadarProcessor.hh"
#include "marine_radar/Types.hh"

namespace marine_radar
{

/// \brief Fill a header with a simulation-time stamp and a frame id.
void FillHeader(double _stamp, const std::string &_frameId,
                gz::msgs::Header &_header);

/// \brief RGB8 PPI image -> gz.msgs.Image (RGB_INT8).
void FillImage(const marine_radar::RgbImage &_img, double _stamp,
               const std::string &_frameId, gz::msgs::Image &_msg);

/// \brief One point per target. Fields (in memory order):
/// world_x, world_y, world_z (FLOAT64);
/// x, y, z, range, bearing, elevation, intensity (FLOAT32);
/// id, age, num_cells (UINT32);
/// min_range, range_extent, bearing_extent, time_offset (FLOAT32).
void FillTargets(const marine_radar::ScanResult &_scan,
                 const std::string &_frameId,
                 gz::msgs::PointCloudPacked &_msg);

/// \brief One point per echo cell. Fields (in memory order):
/// world_x, world_y, world_z (FLOAT64);
/// x, y, z, intensity, range, bearing, elevation, time_offset (FLOAT32).
void FillReturns(const marine_radar::ScanResult &_scan,
                 const std::string &_frameId,
                 gz::msgs::PointCloudPacked &_msg);

}  // namespace marine_radar

#endif  // GZ_SENSORS_MARINE_RADAR_RADARGZMESSAGES_HH_
