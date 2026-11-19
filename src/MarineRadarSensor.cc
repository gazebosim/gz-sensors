/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#if defined(_MSC_VER)
  #pragma warning(push)
  #pragma warning(disable: 4005)
  #pragma warning(disable: 4251)
#endif
#include <gz/msgs/double.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#if defined(_MSC_VER)
  #pragma warning(pop)
#endif

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Event.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/Scene.hh>

#include "gz/sensors/Noise.hh"
#include "gz/sensors/RenderingEvents.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/Util.hh"
#include "gz/sensors/MarineRadarSensor.hh"

#include "marine_radar/RadarConfig.hh"
#include "marine_radar/RadarGzMessages.hh"
#include "marine_radar/RadarProcessor.hh"

using namespace gz;
using namespace sensors;

/// \brief Name of the custom sensor type, as given by the `gz:type`
/// attribute of the `<sensor>` element.
static constexpr const char *kCustomType = "marine_radar";

/// \brief Name of the SDF element holding the radar parameters.
static constexpr const char *kCustomElement = "gz:marine_radar";

namespace
{
/// \brief Convert a steady clock duration to seconds.
double ToSeconds(const std::chrono::steady_clock::duration &_d)
{
  return std::chrono::duration<double>(_d).count();
}

/// \brief Convert a gz-math pose into the radar core representation.
marine_radar::Pose ToCore(const gz::math::Pose3d &_p)
{
  marine_radar::Pose out;
  out.position = {_p.Pos().X(), _p.Pos().Y(), _p.Pos().Z()};
  out.orientation = {_p.Rot().W(), _p.Rot().X(), _p.Rot().Y(), _p.Rot().Z()};
  return out;
}

/// \brief Convert a radar core vector into a gz-math vector.
gz::math::Vector3d ToMath(const marine_radar::Vec3 &_v)
{
  return {_v.x, _v.y, _v.z};
}

/// \brief Convert a gz-math vector into a radar core vector.
marine_radar::Vec3 ToCore(const gz::math::Vector3d &_v)
{
  return {_v.X(), _v.Y(), _v.Z()};
}

/// \brief Trim ASCII whitespace from both ends of a string.
std::string Trimmed(const std::string &_s)
{
  const auto b = _s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos)
    return "";
  const auto e = _s.find_last_not_of(" \t\r\n");
  return _s.substr(b, e - b + 1);
}

/// \brief Parse an SDF boolean written as text.
bool ParseBoolText(const std::string &_text, bool &_out)
{
  std::string t;
  for (char c : _text)
  {
    if (!std::isspace(static_cast<unsigned char>(c)))
    {
      t.push_back(
          static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
  }
  if (t == "true" || t == "1")
  {
    _out = true;
    return true;
  }
  if (t == "false" || t == "0")
  {
    _out = false;
    return true;
  }
  return false;
}

/// \brief How the surface normal of a ray hit is recovered.
enum class NormalEstimation
{
  /// \brief Assume normal incidence (|cos incidence| = 1) for every hit.
  NONE,
  /// \brief Cast two extra probe rays per hit and fit a local plane.
  PROBE
};
}  // namespace

/// \brief Private data for MarineRadarSensor
class gz::sensors::MarineRadarSensor::Implementation
{
  /// \brief Node to create publishers and the command subscriber.
  public: transport::Node node;

  /// \brief Publisher for the PPI image.
  public: transport::Node::Publisher imagePub;

  /// \brief Publisher for the extracted targets.
  public: transport::Node::Publisher targetsPub;

  /// \brief Publisher for the per-cell returns.
  public: transport::Node::Publisher returnsPub;

  /// \brief True if Load() has been called and was successful.
  public: bool initialized = false;

  /// \brief Radar configuration.
  public: marine_radar::RadarConfig config;

  /// \brief Scan generation, echo formation, target extraction and image
  /// rendering. Re-created when the configuration changes at run time.
  public: std::unique_ptr<marine_radar::RadarProcessor> processor;

  /// \brief Ray query used to sample the scene, created from the scene.
  public: rendering::RayQueryPtr rayQuery;

  /// \brief Connection to the scene change event.
  public: common::ConnectionPtr sceneChangeConnection;

  /// \brief True to publish the PPI image.
  public: bool publishImage = true;

  /// \brief True to publish the per-cell returns cloud.
  public: bool publishReturns = true;

  /// \brief How surface normals are recovered from the rendering engine.
  public: NormalEstimation normalEstimation = NormalEstimation::PROBE;

  /// \brief Angular offset of the normal probe rays [rad].
  public: double normalProbeAngle = marine_radar::DegToRad(0.05);

  /// \brief Guards pendingRangeMax, which is written by the transport
  /// callback thread.
  public: std::mutex cmdMutex;

  /// \brief Maximum range requested on the command topic, if any.
  public: std::optional<double> pendingRangeMax;

  /// \brief Simulation time of the previous update [s].
  public: double lastTime = -1.0;

  /// \brief Simulation time the next PPI image is due [s].
  public: double nextImageTime = 0.0;

  /// \brief Number of rays cast during the last update, probe rays included.
  public: unsigned int rayCount = 0;

  /// \brief Scratch buffers, kept to avoid re-allocating every update.
  public: std::vector<marine_radar::RayResult> results;
  public: marine_radar::RgbImage image;
  public: msgs::Image imageMsg;
  public: msgs::PointCloudPacked targetsMsg;
  public: msgs::PointCloudPacked returnsMsg;

  /// \brief True once the missing-scene warning has been printed.
  public: bool warnedNoScene = false;

  /// \brief True once the no-hits warning has been printed.
  public: bool warnedNoHits = false;

  /// \brief Callback for the maximum range command topic.
  public: void OnRangeMax(const msgs::Double &_msg);

  /// \brief Apply a pending maximum range command, if any.
  /// \param[in] _t Current simulation time [s]
  /// \param[in] _name Sensor name, for log messages
  public: void ApplyPendingCommands(double _t, const std::string &_name);

  /// \brief Replace the configuration and restart the radar pipeline.
  /// \param[in] _candidate New configuration
  /// \param[in] _t Current simulation time [s]
  /// \param[in] _name Sensor name, for log messages
  /// \return True if the configuration is valid and was applied
  public: bool Reconfigure(const marine_radar::RadarConfig &_candidate,
              double _t, const std::string &_name);

  /// \brief Cast every planned ray against the scene.
  /// \param[in] _plan Rays to cast, in the radar frame
  /// \param[in] _pose Radar pose in the world
  public: void CastRays(const std::vector<marine_radar::RayRequest> &_plan,
              const marine_radar::Pose &_pose);

  /// \brief Estimate the surface normal at a ray hit by casting two probe
  /// rays slightly off the primary direction and fitting a plane through
  /// the three hit points.
  /// \param[in] _origin Ray origin in the world
  /// \param[in] _dir Unit ray direction in the world
  /// \param[in] _hit Primary hit point in the world
  /// \param[in] _objectId Object the primary ray hit
  /// \param[in] _length Ray length [m]
  /// \return Normal in the world frame, or a zero vector if the normal
  /// could not be recovered (the echo model then assumes normal incidence)
  public: gz::math::Vector3d ProbeNormal(const gz::math::Vector3d &_origin,
              const gz::math::Vector3d &_dir, const gz::math::Vector3d &_hit,
              unsigned int _objectId, double _length);

  /// \brief Read the `<gz:marine_radar>` subtree into a key/value map for
  /// marine_radar::ParseConfig(), consuming the elements handled by this
  /// class on the way.
  /// \param[in] _elem The `<gz:marine_radar>` element
  /// \param[in] _name Sensor name, for log messages
  /// \param[out] _params Parameters for marine_radar::ParseConfig()
  /// \return True if every element handled by this class was valid
  public: bool ReadSdf(const sdf::ElementPtr &_elem, const std::string &_name,
              std::map<std::string, std::string> &_params);

  /// \brief Read one standard SDFormat `<noise>`-shaped element into a
  /// standard deviation.
  /// \param[in] _elem The noise element
  /// \param[in] _key Element name, for log messages
  /// \param[in] _name Sensor name, for log messages
  /// \param[out] _stdDev Standard deviation, in the element's own units
  /// \return True if the element was a supported noise model
  public: bool ReadNoise(const sdf::ElementPtr &_elem,
              const std::string &_key, const std::string &_name,
              double &_stdDev);
};

//////////////////////////////////////////////////
void MarineRadarSensor::Implementation::OnRangeMax(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->pendingRangeMax = _msg.data();
}

//////////////////////////////////////////////////
bool MarineRadarSensor::Implementation::Reconfigure(
    const marine_radar::RadarConfig &_candidate, double _t,
    const std::string &_name)
{
  const auto errors = _candidate.Validate();
  if (!errors.empty())
  {
    gzerr << "[" << _name << "] Rejected radar configuration:" << std::endl;
    for (const auto &e : errors)
      gzerr << "  - " << e << std::endl;
    return false;
  }

  this->config = _candidate;
  this->processor =
      std::make_unique<marine_radar::RadarProcessor>(this->config);
  this->nextImageTime = _t;
  return true;
}

//////////////////////////////////////////////////
void MarineRadarSensor::Implementation::ApplyPendingCommands(
    double _t, const std::string &_name)
{
  std::optional<double> rangeMax;
  {
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    rangeMax.swap(this->pendingRangeMax);
  }
  if (!rangeMax)
    return;

  marine_radar::RadarConfig candidate = this->config;
  candidate.range_max = *rangeMax;
  if (this->Reconfigure(candidate, _t, _name))
  {
    gzmsg << "[" << _name << "] range_max set to " << *rangeMax
          << " m at t=" << _t << " s; radar restarted ("
          << marine_radar::Summary(this->config) << ")" << std::endl;
  }
}

//////////////////////////////////////////////////
gz::math::Vector3d MarineRadarSensor::Implementation::ProbeNormal(
    const gz::math::Vector3d &_origin, const gz::math::Vector3d &_dir,
    const gz::math::Vector3d &_hit, unsigned int _objectId, double _length)
{
  const gz::math::Vector3d zero{0.0, 0.0, 0.0};

  // Two directions perpendicular to the primary ray, so the probes spread
  // the local surface patch in two independent directions.
  const gz::math::Vector3d e1 = _dir.Perpendicular().Normalized();
  const gz::math::Vector3d e2 = _dir.Cross(e1).Normalized();
  const double d = std::tan(this->normalProbeAngle);

  gz::math::Vector3d probe[2];
  for (int k = 0; k < 2; ++k)
  {
    const gz::math::Vector3d dir =
        (_dir + (k == 0 ? e1 : e2) * d).Normalized();
    this->rayQuery->SetOrigin(_origin);
    this->rayQuery->SetDirection(dir);
    const auto res = this->rayQuery->ClosestPoint(false);
    ++this->rayCount;
    // Only a probe landing on the same object describes the same surface;
    // anything else (a miss, an edge, a different object) is discarded and
    // the echo model falls back to normal incidence.
    if (!res || !res.point.IsFinite() || res.objectId != _objectId ||
        (res.point - _origin).Length() >
            _length + this->config.range_resolution)
    {
      return zero;
    }
    probe[k] = res.point;
  }

  const gz::math::Vector3d n =
      (probe[0] - _hit).Cross(probe[1] - _hit);
  if (!n.IsFinite() || n.Length() < 1e-18)
    return zero;
  return n.Normalized();
}

//////////////////////////////////////////////////
void MarineRadarSensor::Implementation::CastRays(
    const std::vector<marine_radar::RayRequest> &_plan,
    const marine_radar::Pose &_pose)
{
  this->results.assign(_plan.size(), marine_radar::RayResult());
  this->rayCount = 0;

  // gz-rendering needs the derived transforms of the scene refreshed once
  // per batch; the remaining queries of the same batch can reuse them.
  bool forceSceneUpdate = true;

  for (size_t i = 0; i < _plan.size(); ++i)
  {
    const marine_radar::Vec3 startW = _pose.Transform(_plan[i].start);
    const marine_radar::Vec3 endW = _pose.Transform(_plan[i].end);
    const gz::math::Vector3d origin = ToMath(startW);
    gz::math::Vector3d dir = ToMath(endW) - origin;
    const double length = dir.Length();
    if (!(length > 0.0))
      continue;
    dir /= length;

    this->rayQuery->SetOrigin(origin);
    this->rayQuery->SetDirection(dir);
    const auto res = this->rayQuery->ClosestPoint(forceSceneUpdate);
    forceSceneUpdate = false;
    ++this->rayCount;

    // RayQueryResult::distance is reported in the hit object's own
    // (unscaled) units by the ogre2 implementation, so the metric hit
    // distance is recomputed from the returned world point instead.
    const double hitDistance =
        res ? (res.point - origin).Length() : -1.0;

    // A miss, or a hit past range_max, is reported to the processor as a
    // non-finite fraction (the "no hit" encoding of the core library).
    if (!res || !res.point.IsFinite() || hitDistance > length)
    {
      this->results[i].fraction = std::nan("");
      continue;
    }

    gz::math::Vector3d normalW{0.0, 0.0, 0.0};
    if (this->normalEstimation == NormalEstimation::PROBE)
    {
      normalW = this->ProbeNormal(origin, dir, res.point, res.objectId,
          length);
    }

    this->results[i].fraction = hitDistance / length;
    this->results[i].point = _pose.InverseTransform(ToCore(res.point));
    // The echo model dots the normal with the ray direction in the radar
    // frame, so rotate the world normal into that frame. A zero vector
    // means "unknown", and the model then assumes normal incidence.
    this->results[i].normal =
        _pose.orientation.RotateReverse(ToCore(normalW));
  }
}

//////////////////////////////////////////////////
bool MarineRadarSensor::Implementation::ReadNoise(
    const sdf::ElementPtr &_elem, const std::string &_key,
    const std::string &_name, double &_stdDev)
{
  // Mirroring e.g. //altimeter/vertical_position, the standard deviation is
  // given by a nested standard SDFormat <noise> element.
  if (!_elem->HasElement("noise"))
  {
    gzerr << "[" << _name << "] <" << _key << "> must contain a standard "
          << "SDFormat <noise> element." << std::endl;
    return false;
  }

  sdf::Noise noiseSdf;
  sdf::Errors errors = noiseSdf.Load(_elem->GetElement("noise"));
  if (!errors.empty())
  {
    for (const auto &e : errors)
      gzerr << "[" << _name << "] <" << _key << ">: " << e << std::endl;
    return false;
  }

  if (noiseSdf.Type() == sdf::NoiseType::NONE)
  {
    _stdDev = 0.0;
    return true;
  }

  if (noiseSdf.Type() != sdf::NoiseType::GAUSSIAN)
  {
    gzerr << "[" << _name << "] <" << _key << "> uses an unsupported noise "
          << "model. The radar echo model only supports 'gaussian' noise."
          << std::endl;
    return false;
  }

  if (std::abs(noiseSdf.Mean()) > 0.0)
  {
    gzwarn << "[" << _name << "] <" << _key << "><mean> is ignored; the "
           << "radar echo model applies zero-mean noise only." << std::endl;
  }

  // NoiseFactory is deliberately not used here: the standard deviation is
  // consumed inside the radar echo model, which applies range, bearing and
  // SNR noise at different stages of the pipeline (and per echo rather than
  // per sample), so a gz::sensors::Noise instance could not be applied to a
  // single published value.
  _stdDev = noiseSdf.StdDev();
  return true;
}

//////////////////////////////////////////////////
bool MarineRadarSensor::Implementation::ReadSdf(
    const sdf::ElementPtr &_elem, const std::string &_name,
    std::map<std::string, std::string> &_params)
{
  bool ok = true;

  // Elements handled by this class rather than by marine_radar::ParseConfig.
  static const std::vector<std::string> sensorKeys = {
    "publish_image", "publish_returns", "normal_estimation",
    "normal_probe_angle_deg", "range_noise", "bearing_noise",
    "intensity_noise"};

  for (auto child = _elem->GetFirstElement(); child;
       child = child->GetNextElement())
  {
    const std::string key = child->GetName();
    if (std::find(sensorKeys.begin(), sensorKeys.end(), key) ==
        sensorKeys.end())
    {
      const std::string value =
          child->GetValue() ? Trimmed(child->GetValue()->GetAsString()) : "";
      if (_params.count(key))
      {
        gzwarn << "[" << _name << "] Parameter <" << key << "> given more "
               << "than once; using the last value." << std::endl;
      }
      _params[key] = value;
      continue;
    }

    if (key == "range_noise")
      ok &= this->ReadNoise(child, key, _name, this->config.range_noise_stddev);
    else if (key == "intensity_noise")
      ok &= this->ReadNoise(child, key, _name,
          this->config.intensity_noise_stddev_db);
    else if (key == "bearing_noise")
    {
      double deg = 0.0;
      if (this->ReadNoise(child, key, _name, deg))
        this->config.bearing_noise_stddev = marine_radar::DegToRad(deg);
      else
        ok = false;
    }
    else if (key == "publish_image" || key == "publish_returns")
    {
      const std::string value =
          child->GetValue() ? child->GetValue()->GetAsString() : "";
      bool flag = true;
      if (!ParseBoolText(value, flag))
      {
        gzerr << "[" << _name << "] <" << key << "> = '" << value
              << "' must be true or false." << std::endl;
        ok = false;
      }
      else if (key == "publish_image")
      {
        this->publishImage = flag;
      }
      else
      {
        this->publishReturns = flag;
      }
    }
    else if (key == "normal_estimation")
    {
      const std::string value = Trimmed(
          child->GetValue() ? child->GetValue()->GetAsString() : "");
      if (value == "probe")
      {
        this->normalEstimation = NormalEstimation::PROBE;
      }
      else if (value == "none")
      {
        this->normalEstimation = NormalEstimation::NONE;
      }
      else
      {
        gzerr << "[" << _name << "] <normal_estimation> = '" << value
              << "' must be 'probe' or 'none'." << std::endl;
        ok = false;
      }
    }
    else if (key == "normal_probe_angle_deg")
    {
      const auto pair = child->Get<double>("", 0.05);
      if (!pair.second || !std::isfinite(pair.first) || pair.first <= 0.0 ||
          pair.first > 5.0)
      {
        gzerr << "[" << _name << "] <normal_probe_angle_deg> must be a "
              << "number in (0, 5] degrees." << std::endl;
        ok = false;
      }
      else
      {
        this->normalProbeAngle = marine_radar::DegToRad(pair.first);
      }
    }
  }

  return ok;
}

//////////////////////////////////////////////////
MarineRadarSensor::MarineRadarSensor()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
MarineRadarSensor::~MarineRadarSensor()
{
  this->dataPtr->sceneChangeConnection.reset();
}

//////////////////////////////////////////////////
bool MarineRadarSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool MarineRadarSensor::Load(const sdf::Sensor &_sdf)
{
  if (!RenderingSensor::Load(_sdf))
    return false;

  // SDFormat has no native radar sensor type, so the marine radar is loaded
  // through the custom sensor mechanism.
  if (_sdf.Type() != sdf::SensorType::CUSTOM)
  {
    gzerr << "Attempting to load a MarineRadar sensor, but received a "
          << _sdf.TypeStr() << ". Use <sensor type=\"custom\" gz:type=\""
          << kCustomType << "\">." << std::endl;
    return false;
  }

  const auto type = customType(_sdf);
  if (kCustomType != type)
  {
    gzerr << "Attempting to load a [" << kCustomType << "] sensor, but got "
          << "type [" << type << "] instead." << std::endl;
    return false;
  }

  if (this->Topic().empty())
    this->SetTopic("/marine_radar");

  // Collect the radar parameters. An absent <gz:marine_radar> element is
  // allowed and gives a fully defaulted 1 km / 24 rpm radar.
  std::map<std::string, std::string> params;
  bool ok = true;
  sdf::ElementPtr elem = _sdf.Element();
  if (elem && elem->HasElement(kCustomElement))
  {
    ok = this->dataPtr->ReadSdf(elem->GetElement(kCustomElement),
        this->Name(), params);
  }
  else
  {
    gzdbg << "No <" << kCustomElement << "> configuration for ["
          << this->Topic() << "]; using defaults." << std::endl;
  }

  auto parsed = marine_radar::ParseConfig(params);
  for (const auto &w : parsed.warnings)
    gzwarn << "[" << this->Name() << "] " << w << std::endl;
  for (const auto &e : parsed.errors)
  {
    gzerr << "[" << this->Name() << "] " << e << std::endl;
    ok = false;
  }
  if (!ok)
  {
    gzerr << "[" << this->Name() << "] Marine radar is not loaded because of "
          << "the configuration errors above." << std::endl;
    return false;
  }

  // The noise elements are written straight into dataPtr->config by
  // ReadSdf(), so carry them over the parsed values.
  const double rangeNoise = this->dataPtr->config.range_noise_stddev;
  const double bearingNoise = this->dataPtr->config.bearing_noise_stddev;
  const double intensityNoise =
      this->dataPtr->config.intensity_noise_stddev_db;
  marine_radar::RadarConfig config = parsed.config;
  if (params.count("range_noise_stddev") == 0)
    config.range_noise_stddev = rangeNoise;
  if (params.count("bearing_noise_stddev_deg") == 0)
    config.bearing_noise_stddev = bearingNoise;
  if (params.count("intensity_noise_stddev_db") == 0)
    config.intensity_noise_stddev_db = intensityNoise;

  if (!this->dataPtr->Reconfigure(config, 0.0, this->Name()))
    return false;

  // Advertise the outputs and subscribe to the run-time command topic.
  this->dataPtr->imagePub =
      this->dataPtr->node.Advertise<msgs::Image>(this->ImageTopic());
  this->dataPtr->targetsPub =
      this->dataPtr->node.Advertise<msgs::PointCloudPacked>(
      this->TargetsTopic());
  this->dataPtr->returnsPub =
      this->dataPtr->node.Advertise<msgs::PointCloudPacked>(
      this->ReturnsTopic());
  if (!this->dataPtr->imagePub || !this->dataPtr->targetsPub ||
      !this->dataPtr->returnsPub)
  {
    gzerr << "Unable to create publishers on topic[" << this->Topic()
          << "/{image,targets,returns}].\n";
    return false;
  }

  const std::string cmdTopic = this->Topic() + "/cmd/range_max";
  if (!this->dataPtr->node.Subscribe(cmdTopic,
      &MarineRadarSensor::Implementation::OnRangeMax, this->dataPtr.get()))
  {
    gzwarn << "[" << this->Name() << "] Could not subscribe to " << cmdTopic
           << "; run-time range changes are unavailable." << std::endl;
  }

  if (this->Scene())
    this->dataPtr->rayQuery = this->Scene()->CreateRayQuery();

  this->dataPtr->sceneChangeConnection =
      RenderingEvents::ConnectSceneChangeCallback(
      std::bind(&MarineRadarSensor::SetScene, this, std::placeholders::_1));

  gzmsg << "Marine radar for [" << this->Name() << "] advertised on ["
        << this->Topic() << "/{image,targets,returns}]: "
        << marine_radar::Summary(this->dataPtr->config) << std::endl;

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool MarineRadarSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
void MarineRadarSensor::SetScene(rendering::ScenePtr _scene)
{
  if (this->Scene() == _scene)
    return;

  this->dataPtr->rayQuery.reset();
  RenderingSensor::SetScene(_scene);
  if (_scene)
    this->dataPtr->rayQuery = _scene->CreateRayQuery();

  // The scene the echoes were collected from is gone.
  this->Reset();
}

//////////////////////////////////////////////////
bool MarineRadarSensor::Update(
    const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("MarineRadarSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->rayQuery)
  {
    if (!this->dataPtr->warnedNoScene)
    {
      this->dataPtr->warnedNoScene = true;
      gzwarn << "[" << this->Name() << "] No rendering scene available; the "
             << "marine radar cannot cast rays." << std::endl;
    }
    return false;
  }
  this->dataPtr->warnedNoScene = false;

  const double t = ToSeconds(_now);
  if (t < this->dataPtr->lastTime)
  {
    // Simulation time went backwards, e.g. a world reset.
    this->Reset();
  }
  this->dataPtr->lastTime = t;
  this->dataPtr->ApplyPendingCommands(t, this->Name());

  const marine_radar::Pose pose = ToCore(this->Pose());
  auto &processor = *this->dataPtr->processor;

  // 1. Ask the pipeline which rays are due, 2. cast them, 3. hand the hits
  // back. The pipeline owns the antenna schedule, so the sensor update rate
  // only has to be high enough to keep up with the spoke rate.
  {
    GZ_PROFILE("MarineRadarSensor::Update::RayCast");
    const auto &plan = processor.PlanRays(t, pose);
    this->dataPtr->CastRays(plan, pose);
    processor.SetRayResults(this->dataPtr->results, pose);
  }

  // 4. Form the spokes that are due and publish every completed scan.
  const auto scans = processor.Process(t, pose);
  for (const auto &scan : scans)
  {
    marine_radar::FillTargets(scan, this->FrameId(),
        this->dataPtr->targetsMsg);
    this->AddSequence(this->dataPtr->targetsMsg.mutable_header(), "targets");
    this->dataPtr->targetsPub.Publish(this->dataPtr->targetsMsg);

    if (this->dataPtr->publishReturns)
    {
      marine_radar::FillReturns(scan, this->FrameId(),
          this->dataPtr->returnsMsg);
      this->AddSequence(this->dataPtr->returnsMsg.mutable_header(),
          "returns");
      this->dataPtr->returnsPub.Publish(this->dataPtr->returnsMsg);
    }

    const auto &stats = processor.Stats();
    if (!this->dataPtr->warnedNoHits && stats.scans_completed >= 2 &&
        stats.ray_hits == 0)
    {
      this->dataPtr->warnedNoHits = true;
      gzwarn << "[" << this->Name() << "] No ray hit anything during the "
             << "first two scans. If obstacles are inside the radar range, "
             << "check that they are part of the rendering scene and that "
             << "the sensor pose is above the sea surface." << std::endl;
    }
  }

  // 5. Render the PPI image when it is due.
  if (this->dataPtr->publishImage && this->dataPtr->config.image_rate > 0.0 &&
      t >= this->dataPtr->nextImageTime)
  {
    GZ_PROFILE("MarineRadarSensor::Update::RenderImage");
    processor.RenderImage(t, pose, this->dataPtr->image);
    marine_radar::FillImage(this->dataPtr->image, t, this->FrameId(),
        this->dataPtr->imageMsg);
    this->AddSequence(this->dataPtr->imageMsg.mutable_header(), "image");
    this->dataPtr->imagePub.Publish(this->dataPtr->imageMsg);

    const double period = 1.0 / this->dataPtr->config.image_rate;
    this->dataPtr->nextImageTime += period;
    if (this->dataPtr->nextImageTime <= t)
      this->dataPtr->nextImageTime = t + period;
  }

  return true;
}

//////////////////////////////////////////////////
void MarineRadarSensor::Reset()
{
  if (this->dataPtr->processor)
    this->dataPtr->processor->Reset();
  this->dataPtr->nextImageTime = std::max(0.0, this->dataPtr->lastTime);
  this->dataPtr->warnedNoHits = false;
}

//////////////////////////////////////////////////
bool MarineRadarSensor::HasConnections() const
{
  return (this->dataPtr->imagePub && this->dataPtr->imagePub.HasConnections())
      || (this->dataPtr->targetsPub &&
          this->dataPtr->targetsPub.HasConnections())
      || (this->dataPtr->returnsPub &&
          this->dataPtr->returnsPub.HasConnections());
}

//////////////////////////////////////////////////
double MarineRadarSensor::RangeMin() const
{
  return this->dataPtr->config.range_min;
}

//////////////////////////////////////////////////
double MarineRadarSensor::RangeMax() const
{
  return this->dataPtr->config.range_max;
}

//////////////////////////////////////////////////
bool MarineRadarSensor::SetRangeMax(double _range)
{
  marine_radar::RadarConfig candidate = this->dataPtr->config;
  candidate.range_max = _range;
  return this->dataPtr->Reconfigure(candidate,
      std::max(0.0, this->dataPtr->lastTime), this->Name());
}

//////////////////////////////////////////////////
double MarineRadarSensor::RangeResolution() const
{
  return this->dataPtr->config.range_resolution;
}

//////////////////////////////////////////////////
double MarineRadarSensor::HorizontalFOV() const
{
  return this->dataPtr->config.horizontal_fov;
}

//////////////////////////////////////////////////
double MarineRadarSensor::AngularResolution() const
{
  return this->dataPtr->config.angular_resolution;
}

//////////////////////////////////////////////////
double MarineRadarSensor::RotationRateRpm() const
{
  return this->dataPtr->config.rotation_rate_rpm;
}

//////////////////////////////////////////////////
unsigned int MarineRadarSensor::SpokesPerScan() const
{
  return static_cast<unsigned int>(this->dataPtr->config.NumSpokes());
}

//////////////////////////////////////////////////
unsigned int MarineRadarSensor::RangeBins() const
{
  return static_cast<unsigned int>(this->dataPtr->config.NumBins());
}

//////////////////////////////////////////////////
unsigned int MarineRadarSensor::ImageSize() const
{
  return static_cast<unsigned int>(this->dataPtr->config.image_size);
}

//////////////////////////////////////////////////
uint64_t MarineRadarSensor::ScanCount() const
{
  return this->dataPtr->processor ?
      this->dataPtr->processor->Stats().scans_completed : 0u;
}

//////////////////////////////////////////////////
unsigned int MarineRadarSensor::TargetCount() const
{
  return this->dataPtr->processor ?
      static_cast<unsigned int>(
          this->dataPtr->processor->LastTargets().size()) : 0u;
}

//////////////////////////////////////////////////
unsigned int MarineRadarSensor::RayCount() const
{
  return this->dataPtr->rayCount;
}

//////////////////////////////////////////////////
std::string MarineRadarSensor::ImageTopic() const
{
  return this->Topic() + "/image";
}

//////////////////////////////////////////////////
std::string MarineRadarSensor::TargetsTopic() const
{
  return this->Topic() + "/targets";
}

//////////////////////////////////////////////////
std::string MarineRadarSensor::ReturnsTopic() const
{
  return this->Topic() + "/returns";
}
