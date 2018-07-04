/*
 * copyright (c) 2017 open source robotics foundation
 *
 * licensed under the apache license, version 2.0 (the "license");
 * you may not use this file except in compliance with the license.
 * you may obtain a copy of the license at
 *
 *     http://www.apache.org/licenses/license-2.0
 *
 * unless required by applicable law or agreed to in writing, software
 * distributed under the license is distributed on an "as is" basis,
 * without warranties or conditions of any kind, either express or implied.
 * see the license for the specific language governing permissions and
 * limitations under the license.
 *
*/

#include <ignition/sensors/Lidar.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>
#include <ignition/math/Angle.hh>
#include <ignition/sensors/Events.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/transport.hh>

using namespace ignition::sensors;


class ignition::sensors::LidarPrivate
{
  /// \brief constructor
  public: LidarPrivate();

  /// \brief destructor
  public: ~LidarPrivate();

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher pub;

  /// \brief Laser message to publish data.
  public: ignition::msgs::LaserScan laserMsg;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Event triggered when new laser range data are available.
  /// \param[in] _frame New frame containing raw laser data.
  /// \param[in] _width Width of frame.
  /// \param[in] _height Height of frame.
  /// \param[in] _depth Depth of frame.
  /// \param[in] _format Format of frame.
  public: ignition::common::EventT<void(const float *_frame,
               unsigned int _width, unsigned int _height,
               unsigned int _depth, const std::string &_format)>
               dataEvent;

  /// \brief Raw buffer of laser data.
  public: float *laserBuffer = nullptr;

  /// \brief Horizontal ray count.
  public: unsigned int horzRayCount = 0;

  /// \brief Vertical ray count.
  public: unsigned int vertRayCount = 0;

  /// \brief Horizontal range count.
  public: unsigned int horzRangeCount = 0;

  /// \brief Vertical range count.
  public: unsigned int vertRangeCount = 0;

  /// \brief Range count ratio.
  public: double rangeCountRatio = 0;

  /// \brief The minimum range.
  public: double rangeMin = 0;

  /// \brief The maximum range.
  public: double rangeMax = 0;

  /// \brief Scan SDF element.
  public: sdf::ElementPtr scanElem;

  /// \brief Horizontal SDF element.
  public: sdf::ElementPtr horzElem;

  /// \brief Vertical SDF element.
  public: sdf::ElementPtr vertElem;

  /// \brief Range SDF element.
  public: sdf::ElementPtr rangeElem;

  /// \brief Camera SDF element.
  public: sdf::ElementPtr cameraElem;
};

//////////////////////////////////////////////////
LidarPrivate::LidarPrivate()
{
}

//////////////////////////////////////////////////
LidarPrivate::~LidarPrivate()
{
}

//////////////////////////////////////////////////
Lidar::Lidar()
  : dataPtr(new LidarPrivate())
{
}

//////////////////////////////////////////////////
Lidar::~Lidar()
{
  this->Fini();
}

//////////////////////////////////////////////////
bool Lidar::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
void Lidar::Fini()
{
}

//////////////////////////////////////////////////
bool Lidar::Load(sdf::ElementPtr _sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Validate that SDF has a sensor ray on it
  if (_sdf->GetName() == "sensor")
  {
    if (!_sdf->GetElement("ray"))
    {
      ignerr << "<sensor><camera> SDF element not found while attempting to "
        << "load a ignition::sensors::Lidar\n";
      return false;
    }
  }

  // Load sensor element
  if (!this->Sensor::Load(_sdf))
  {
    return false;
  }

  // Register publisher
  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::LaserScan>(
        this->Topic());
  if (!this->dataPtr->pub)
  {
    return false;
  }

  // Load ray atributes
  sdf::ElementPtr rayElem = this->SDF()->GetElement("ray");
  this->dataPtr->scanElem = rayElem->GetElement("scan");
  this->dataPtr->horzElem = this->dataPtr->scanElem->GetElement("horizontal");
  this->dataPtr->rangeElem = rayElem->GetElement("range");

  if (this->dataPtr->scanElem->HasElement("vertical"))
    this->dataPtr->vertElem = this->dataPtr->scanElem->GetElement("vertical");

  this->dataPtr->horzRayCount = this->RayCount();
  this->dataPtr->vertRayCount = this->VerticalRayCount();

  if (this->dataPtr->horzRayCount == 0 || this->dataPtr->vertRayCount == 0)
  {
    ignerr << "Lidar: Image has 0 size!\n";
  }

  this->dataPtr->horzRangeCount = this->RangeCount();
  this->dataPtr->vertRangeCount = this->VerticalRangeCount();

  // Handle noise model settings.
  // if (rayElem->HasElement("noise"))
  // {
  //   this->noises[GPU_RAY_NOISE] =
  //       NoiseFactory::NewNoiseModel(rayElem->GetElement("noise"),
  //       this->Type());
  // }

  // this->dataPtr->parentEntity =
  //   this->world->EntityByName(this->ParentName());
  // GZ_ASSERT(this->dataPtr->parentEntity != nullptr,
  //     "Unable to get the parent entity.");

  this->dataPtr->initialized = true;
  return true;
}

/////////////////////////////////////////////////
ignition::common::ConnectionPtr Lidar::ConnectNewLaserFrame(
          std::function<void(const float *, unsigned int, unsigned int,
            unsigned int, const std::string &)> _subscriber)
{
  return this->dataPtr->dataEvent.Connect(_subscriber);
}

//////////////////////////////////////////////////
bool Lidar::Update(const common::Time &/*_now*/)
{
  return true;
}

//////////////////////////////////////////////////
bool Lidar::PublishLaserScan(const common::Time &_now)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!this->dataPtr->initialized)
    return false;

  // create message
  this->dataPtr->laserMsg.mutable_header()->mutable_stamp()->set_sec(
      _now.sec);
  this->dataPtr->laserMsg.mutable_header()->mutable_stamp()->set_nsec(
      _now.nsec);

  // Store the latest laser scans into laserMsg
  // msgs::Set(this->dataPtr->laserMsg.mutable_world_pose(),
  //     this->Pose() + this->dataPtr->parentEntity->WorldPose());
  this->dataPtr->laserMsg.set_angle_min(this->AngleMin().Radian());
  this->dataPtr->laserMsg.set_angle_max(this->AngleMax().Radian());
  this->dataPtr->laserMsg.set_angle_step(this->AngleResolution());
  this->dataPtr->laserMsg.set_count(this->RangeCount());

  this->dataPtr->laserMsg.set_vertical_angle_min(
      this->VerticalAngleMin().Radian());
  this->dataPtr->laserMsg.set_vertical_angle_max(
      this->VerticalAngleMax().Radian());
  this->dataPtr->laserMsg.set_vertical_angle_step(
      this->VerticalAngleResolution());
  this->dataPtr->laserMsg.set_vertical_count(
      this->dataPtr->vertRangeCount);

  this->dataPtr->laserMsg.set_range_min(this->dataPtr->rangeMin);
  this->dataPtr->laserMsg.set_range_max(this->dataPtr->rangeMax);

  const int numRays = this->RayCount() * this->VerticalRayCount();
  if (this->dataPtr->laserMsg.ranges_size() != numRays)
  {
    // igndbg << "Size mismatch; allocating memory\n";
    this->dataPtr->laserMsg.clear_ranges();
    this->dataPtr->laserMsg.clear_intensities();
    for (int i = 0; i < numRays; ++i)
    {
      this->dataPtr->laserMsg.add_ranges(ignition::math::NAN_F);
      this->dataPtr->laserMsg.add_intensities(ignition::math::NAN_F);
    }
  }

  for (unsigned int j = 0; j < this->dataPtr->vertRangeCount; ++j)
  {
    for (unsigned int i = 0; i < this->dataPtr->horzRangeCount; ++i)
    {
      int index = j * this->dataPtr->horzRangeCount + i;
      double range = this->dataPtr->laserBuffer[index*3];

      // Mask ranges outside of min/max to +/- inf, as per REP 117
      if (range >= this->dataPtr->rangeMax)
      {
        range = IGN_DBL_INF;
      }
      else if (range <= this->dataPtr->rangeMin)
      {
        range = -IGN_DBL_INF;
      }
      // TODO(jchoclin): add LIDAR_NOISE
      // else if (this->noises.find(GPU_RAY_NOISE) !=
      //          this->noises.end())
      // {
      //   range = this->noises[GPU_RAY_NOISE]->Apply(range);
      //   range = ignition::math::clamp(range,
      //       this->dataPtr->rangeMin, this->dataPtr->rangeMax);
      // }

      range = ignition::math::isnan(range) ? this->dataPtr->rangeMax : range;
      this->dataPtr->laserMsg.set_ranges(index, range);
      this->dataPtr->laserMsg.set_intensities(index,
          this->dataPtr->laserBuffer[index * 3 + 1]);
    }
  }

  // publish
  this->dataPtr->pub.Publish(this->dataPtr->laserMsg);

  return true;
}

//////////////////////////////////////////////////
bool Lidar::IsHorizontal() const
{
//  return this->dataPtr->laserCam->IsHorizontal();
  return 0;
}

//////////////////////////////////////////////////
double Lidar::RangeCountRatio() const
{
  return this->dataPtr->rangeCountRatio;
}

//////////////////////////////////////////////////
ignition::math::Angle Lidar::AngleMin() const
{
  return this->dataPtr->horzElem->Get<double>("min_angle");
}

//////////////////////////////////////////////////
void Lidar::SetAngleMin(double _angle)
{
  this->dataPtr->horzElem->GetElement("min_angle")->Set(_angle);
}

//////////////////////////////////////////////////
ignition::math::Angle Lidar::AngleMax() const
{
  return this->dataPtr->horzElem->Get<double>("max_angle");
}

//////////////////////////////////////////////////
void Lidar::SetAngleMax(double _angle)
{
  this->dataPtr->horzElem->GetElement("max_angle")->Set(_angle);
}

//////////////////////////////////////////////////
double Lidar::RangeMin() const
{
  return this->dataPtr->rangeElem->Get<double>("min");
}

//////////////////////////////////////////////////
double Lidar::RangeMax() const
{
  return this->dataPtr->rangeElem->Get<double>("max");
}

/////////////////////////////////////////////////
double Lidar::AngleResolution() const
{
  return (this->AngleMax() - this->AngleMin()).Radian() /
    (this->RangeCount()-1);
}

//////////////////////////////////////////////////
double Lidar::RangeResolution() const
{
  return this->dataPtr->rangeElem->Get<double>("resolution");
}

//////////////////////////////////////////////////
int Lidar::RayCount() const
{
  return this->dataPtr->horzElem->Get<unsigned int>("samples");
}

//////////////////////////////////////////////////
int Lidar::RangeCount() const
{
  return this->RayCount() * this->dataPtr->horzElem->Get<double>("resolution");
}

//////////////////////////////////////////////////
int Lidar::VerticalRayCount() const
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    return this->dataPtr->vertElem->Get<unsigned int>("samples");
  else
    return 1;
}

//////////////////////////////////////////////////
int Lidar::VerticalRangeCount() const
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
  {
    int rows =  (this->VerticalRayCount() *
          this->dataPtr->vertElem->Get<double>("resolution"));
    if (rows > 1)
      return rows;
    else
      return 1;
  }
  else
    return 1;
}

//////////////////////////////////////////////////
ignition::math::Angle Lidar::VerticalAngleMin() const
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    return this->dataPtr->vertElem->Get<double>("min_angle");
  else
    return ignition::math::Angle(0);
}

//////////////////////////////////////////////////
void Lidar::SetVerticalAngleMin(const double _angle)
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    this->dataPtr->vertElem->GetElement("min_angle")->Set(_angle);
}

//////////////////////////////////////////////////
ignition::math::Angle Lidar::VerticalAngleMax() const
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    return this->dataPtr->vertElem->Get<double>("max_angle");
  else
    return ignition::math::Angle(0);
}

//////////////////////////////////////////////////
double Lidar::VerticalAngleResolution() const
{
  return (this->VerticalAngleMax() - this->VerticalAngleMin()).Radian() /
    (this->VerticalRangeCount()-1);
}

//////////////////////////////////////////////////
void Lidar::SetVerticalAngleMax(const double _angle)
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    this->dataPtr->vertElem->GetElement("max_angle")->Set(_angle);
}

//////////////////////////////////////////////////
void Lidar::Ranges(std::vector<double> &_ranges) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  _ranges.resize(this->dataPtr->laserMsg.ranges_size());
  memcpy(&_ranges[0], this->dataPtr->laserMsg.ranges().data(),
         sizeof(_ranges[0]) * this->dataPtr->laserMsg.ranges_size());
}

//////////////////////////////////////////////////
double Lidar::Range(const int _index) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->laserMsg.ranges_size() == 0)
  {
    ignwarn << "ranges not constructed yet (zero sized)\n";
    return 0.0;
  }
  if (_index < 0 || _index > this->dataPtr->laserMsg.ranges_size())
  {
    ignerr << "Invalid range index[" << _index << "]\n";
    return 0.0;
  }

  return this->dataPtr->laserMsg.ranges(_index);
}

//////////////////////////////////////////////////
double Lidar::Retro(const int /*_index*/) const
{
  return 0.0;
}

//////////////////////////////////////////////////
int Lidar::Fiducial(const unsigned int /*_index*/) const
{
  return -1;
}


//////////////////////////////////////////////////
bool Lidar::IsActive() const
{
//  return Sensor::IsActive() ||
//    (this->dataPtr->pub && this->dataPtr->pub->HasConnections());
  return true;
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(
    ignition::sensors::Lidar,
    ignition::sensors::Sensor)
