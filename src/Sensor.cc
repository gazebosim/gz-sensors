/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include "ignition/sensors/Sensor.hh"

#include <chrono>
#include <map>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/transport/Node.hh>
#include <ignition/transport/TopicUtils.hh>

using namespace ignition::sensors;

class ignition::sensors::SensorPrivate
{
  /// \brief Populates fields from a <sensor> DOM
  public: bool PopulateFromSDF(const sdf::Sensor &_sdf);

  /// \brief Set topic where sensor data is published.
  /// \param[in] _topic Topic sensor publishes data to.
  /// \return True if a valid topic was set.
  public: bool SetTopic(const std::string &_topic);

  /// \brief Publishes information about the performance of the sensor.
  /// \param[in] _now Current simulation time.
  public: void PublishMetrics(const std::chrono::duration<double> &_now);

  /// \brief Set the rate on which the sensor should publish its data. This
  /// method doesn't allow to set a higher rate than what is in the SDF.
  /// \param[in] _rate Maximum rate of the sensor. It is capped by the
  /// <update_rate> value from SDF, and zero is allowed only when zero is also
  /// in SDF.
  /// \return True if a valid topic was set.
  public: void SetRate(const ignition::msgs::Double &_rate);

  /// \brief id given to sensor when constructed
  public: SensorId id;

  /// \brief Counter used to generate unique sensor identifiers.
  public: static SensorId idCounter;

  /// \brief name given to sensor when loaded
  public: std::string name;

  /// \brief name given to the sensor parent link
  public: std::string parent;

  /// \brief topic to send sensor data
  public: std::string topic;

  /// \brief Pose of the sensor
  public: ignition::math::Pose3d pose;

  /// \brief Flag to enable publishing performance metrics.
  public: bool enableMetrics{false};

  /// \brief How many times the sensor will generate data per second (value from
  /// SDF.
  public: double sdfUpdateRate = 0.0;

  /// \brief How many times the sensor will generate data per second (currently
  /// used value).
  public: double updateRate = 0.0;

  /// \brief What sim time should this sensor update at
  public: std::chrono::steady_clock::duration nextUpdateTime
    {std::chrono::steady_clock::duration::zero()};

  /// \brief Last steady clock time reading from last Update call.
  public: std::chrono::time_point<std::chrono::steady_clock> lastRealTime;

  /// \brief Last sim time at Update call.
  public: std::chrono::duration<double> lastUpdateTime{0};

  /// \brief Transport node.
  public: ignition::transport::Node node;

  /// \brief Publishes the PerformanceSensorMetrics message.
  public: ignition::transport::Node::Publisher performanceSensorMetricsPub;

  /// \brief SDF element with sensor information.
  public: sdf::ElementPtr sdf = nullptr;

  /// \brief SDF Sensor DOM object.
  public: sdf::Sensor sdfSensor;

  /// \brief Sequence numbers that are used in sensor data message headers.
  /// A map is used so that a single sensor can have multiple sensor
  /// streams each with a sequence counter.
  public: std::map<std::string, uint64_t> sequences;
};

SensorId SensorPrivate::idCounter = 0;

//////////////////////////////////////////////////
bool SensorPrivate::PopulateFromSDF(const sdf::Sensor &_sdf)
{
  this->sdfSensor = _sdf;

  // All SDF code gets auto converted to latest version. This code is
  // written assuming sdformat 1.7 is the latest

  // \todo(nkoenig) what to do with <always_on>? SDFormat docs seem
  // to say if true
  //  then the update_rate will be obeyed. Gazebo seems to use it as if
  //  true means enable the sensor at startup.

  // \todo(nkoenig) what to do with <visualize>? ign-sensor data is meant to
  // create
  // sensor data. Whether or not that data should be visualized seems to
  // be outside the scope of this library

  // \todo(nkoenig) how to use frame?
  this->name = _sdf.Name();
  if (!_sdf.Topic().empty())
  {
    if (!this->SetTopic(_sdf.Topic()))
      return false;
  }

  // Try resolving the pose first, and only use the raw pose if that fails
  auto semPose = _sdf.SemanticPose();
  sdf::Errors errors = semPose.Resolve(this->pose);
  if (!errors.empty())
  {
    this->pose = _sdf.RawPose();
  }

  this->sdfUpdateRate = this->updateRate = _sdf.UpdateRate();

  this->enableMetrics = _sdf.EnableMetrics();
  return true;
}

//////////////////////////////////////////////////
Sensor::Sensor() :
  dataPtr(new SensorPrivate)
{
  this->dataPtr->id = (++this->dataPtr->idCounter);
}

//////////////////////////////////////////////////
bool Sensor::Init()
{
  this->dataPtr->nextUpdateTime = std::chrono::steady_clock::duration::zero();
  return true;
}

//////////////////////////////////////////////////
Sensor::~Sensor()
{
}

//////////////////////////////////////////////////
bool Sensor::Load(const sdf::Sensor &_sdf)
{
  const bool success = this->dataPtr->PopulateFromSDF(_sdf);
  if (!success)
    return false;

  auto sensorTopic = this->Topic();
  if (sensorTopic.empty())
    sensorTopic = "/" + this->Name();

  const auto rateTopic = sensorTopic + "/set_rate";

  if (!this->dataPtr->node.Advertise(rateTopic,
    &SensorPrivate::SetRate, this->dataPtr.get()))
  {
    ignerr << "Unable to create service server on topic["
           << rateTopic << "].\n";
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool Sensor::Load(sdf::ElementPtr _sdf)
{
  if (!this->dataPtr->sdf)
  {
    this->dataPtr->sdf = _sdf->Clone();
  }
  else
    this->dataPtr->sdf->Copy(_sdf);

  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
sdf::ElementPtr Sensor::SDF() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
SensorId Sensor::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
std::string Sensor::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
std::string Sensor::Topic() const
{
  return this->dataPtr->topic;
}

//////////////////////////////////////////////////
bool Sensor::SetTopic(const std::string &_topic)
{
  return this->dataPtr->SetTopic(_topic);
}

//////////////////////////////////////////////////
bool SensorPrivate::SetTopic(const std::string &_topic)
{
  auto validTopic = transport::TopicUtils::AsValidTopic(_topic);
  if (validTopic.empty())
  {
    ignerr << "Failed to set sensor topic [" << _topic << "]" << std::endl;
    return false;
  }

  this->topic = validTopic;
  return true;
}

//////////////////////////////////////////////////
bool Sensor::EnableMetrics() const
{
  return this->dataPtr->enableMetrics;
}


//////////////////////////////////////////////////
void Sensor::SetEnableMetrics(bool _enableMetrics)
{
  this->dataPtr->enableMetrics = _enableMetrics;
}

//////////////////////////////////////////////////
void Sensor::PublishMetrics(const std::chrono::duration<double> &_now)
{
  return this->dataPtr->PublishMetrics(_now);
}

//////////////////////////////////////////////////
void SensorPrivate::PublishMetrics(const std::chrono::duration<double> &_now)
{
  if(!this->performanceSensorMetricsPub)
  {
    const auto validTopic = transport::TopicUtils::AsValidTopic(
      this->topic + "/performance_metrics");
    if (validTopic.empty())
    {
      ignerr << "Failed to set metrics sensor topic [" << topic << "]" <<
        std::endl;
      return;
    }
    this->performanceSensorMetricsPub =
      node.Advertise<msgs::PerformanceSensorMetrics>(validTopic);
  }
  if (!performanceSensorMetricsPub ||
      !performanceSensorMetricsPub.HasConnections())
  {
    return;
  }

  // Computes simulation update rate and real update rate.
  double simUpdateRate;
  double realUpdateRate;
  const auto clockNow = std::chrono::steady_clock::now();
  // If lastUpdateTime == 0 means it wasn't initialized yet.
  if(this->lastUpdateTime.count() > 0)
  {
    const double diffSimUpdate = _now.count() -
      this->lastUpdateTime.count();
    simUpdateRate = 1.0 / diffSimUpdate;
    const double diffRealUpdate =
      std::chrono::duration_cast<std::chrono::duration<double>>(
        clockNow - this->lastRealTime).count();
    realUpdateRate = diffRealUpdate < std::numeric_limits<double>::epsilon() ?
      std::numeric_limits<double>::infinity() : 1.0 / diffRealUpdate;
  }

  // Update last time values.
  this->lastUpdateTime = _now;
  this->lastRealTime = clockNow;

  // Fill performance sensor metrics message.
  msgs::PerformanceSensorMetrics performanceSensorMetricsMsg;
  performanceSensorMetricsMsg.set_name(this->name);
  performanceSensorMetricsMsg.set_real_update_rate(realUpdateRate);
  performanceSensorMetricsMsg.set_sim_update_rate(simUpdateRate);
  performanceSensorMetricsMsg.set_nominal_update_rate(this->updateRate);

  // Publish data
  performanceSensorMetricsPub.Publish(performanceSensorMetricsMsg);
}

//////////////////////////////////////////////////
void SensorPrivate::SetRate(const ignition::msgs::Double &_rate)
{
  auto rate = _rate.data();
  if (rate < 0.0)
    rate = 0.0;

  // if SDF has zero, any value can be set; for non-zero SDF values, we need to
  // check whether they are in bounds, i.e. greater than zero and lower or equal
  // to the SDF value
  if (!ignition::math::lessOrNearEqual(this->sdfUpdateRate, 0.0))
  {
    if (ignition::math::lessOrNearEqual(rate, 0.0))
    {
      ignerr << "Cannot set update rate of sensor " << this->name << " to zero "
             << "because the <update_rate> SDF element is non-zero."
             << std::endl;
      return;
    }
    // apply the upper rate limit from SDF
    else if (!ignition::math::lessOrNearEqual(rate, this->sdfUpdateRate))
    {
      ignerr << "Trying to set update rate of sensor " << this->name << " to "
             << rate << ", but the maximum rate in <update_rate> SDF element "
             << "is " << this->sdfUpdateRate << ". Ignoring the request."
             << std::endl;
      return;
    }
  }

  igndbg << "Setting update rate of sensor " << this->name << " to " << rate
         << " Hz" << std::endl;

  this->updateRate = rate;
}

//////////////////////////////////////////////////
ignition::math::Pose3d Sensor::Pose() const
{
  return this->dataPtr->pose;
}

//////////////////////////////////////////////////
std::string Sensor::Parent() const
{
  return this->dataPtr->parent;
}

//////////////////////////////////////////////////
void Sensor::SetParent(const std::string &_parent)
{
  this->dataPtr->parent = _parent;
}

//////////////////////////////////////////////////
void Sensor::SetPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

//////////////////////////////////////////////////
double Sensor::UpdateRate() const
{
  return this->dataPtr->updateRate;
}

//////////////////////////////////////////////////
void Sensor::SetUpdateRate(const double _hz)
{
  if (_hz < 0)
  {
    this->dataPtr->updateRate = 0;
  }
  else
  {
    this->dataPtr->updateRate = _hz;
  }
}

//////////////////////////////////////////////////
bool Sensor::Update(const std::chrono::steady_clock::duration &_now,
                  const bool _force)
{
  IGN_PROFILE("Sensor::Update");
  bool result = false;

  // Check if it's time to update
  if (_now < this->dataPtr->nextUpdateTime && !_force &&
      this->dataPtr->updateRate > 0)
  {
    return result;
  }

  // Make the update happen
  result = this->Update(_now);

  // Publish metrics
  if (this->EnableMetrics())
  {
    auto secs = std::chrono::duration_cast<std::chrono::duration<double>>(_now);
    this->PublishMetrics(secs);
  }

  if (!_force && this->dataPtr->updateRate > 0.0)
  {
    // Update the time the plugin should be loaded
    auto delta = std::chrono::duration_cast< std::chrono::milliseconds>
      (std::chrono::duration< double >(1.0 / this->dataPtr->updateRate));
    this->dataPtr->nextUpdateTime += delta;
  }

  return result;
}

//////////////////////////////////////////////////
bool Sensor::Update(const ignition::common::Time &_now, const bool _force)
{
  return this->Update(math::secNsecToDuration(_now.sec, _now.nsec), _force);
}

//////////////////////////////////////////////////
std::chrono::steady_clock::duration Sensor::NextDataUpdateTime() const
{
  return this->dataPtr->nextUpdateTime;
}

//////////////////////////////////////////////////
ignition::common::Time Sensor::NextUpdateTime() const
{
  std::pair<uint64_t, uint64_t> secNsec =
    math::durationToSecNsec(this->dataPtr->nextUpdateTime);
  return common::Time(secNsec.first, secNsec.second);
}

/////////////////////////////////////////////////
void Sensor::AddSequence(ignition::msgs::Header *_msg,
                         const std::string &_seqKey)
{
  std::string value = "0";

  if (this->dataPtr->sequences.find(_seqKey) == this->dataPtr->sequences.end())
    this->dataPtr->sequences[_seqKey] = 0;
  else
    value = std::to_string(++this->dataPtr->sequences[_seqKey]);

  // Set the value if a `sequence` key already exists.
  for (int index = 0; index < _msg->data_size(); ++index)
  {
    if (_msg->data(index).key() == "seq")
    {
      if (_msg->data(index).value_size() == 0)
        _msg->mutable_data(index)->add_value(value);
      else
        _msg->mutable_data(index)->set_value(0, value);
      return;
    }
  }

  // Otherwise, add the sequence key-value pair.
  ignition::msgs::Header::Map *map = _msg->add_data();
  map->set_key("seq");
  map->add_value(value);
}
