/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <mutex>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Frustum.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/sensors/SensorFactory.hh"
#include "ignition/sensors/LogicalCameraSensor.hh"

using namespace ignition;
using namespace sensors;

/// \brief Private data for LogicalCameraSensor
class ignition::sensors::LogicalCameraSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish logical camera messages.
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief Camera frustum.
  public: ignition::math::Frustum frustum;

  /// \brief Set world pose.
  public: math::Pose3d worldPose;

  /// \brief List of models in the world
  public: std::map<std::string, math::Pose3d> models;

  /// \brief Msg containg info on models detected by logical camera
  ignition::msgs::LogicalCameraImage msg;
};

//////////////////////////////////////////////////
LogicalCameraSensor::LogicalCameraSensor()
  : dataPtr(new LogicalCameraSensorPrivate())
{
}

//////////////////////////////////////////////////
LogicalCameraSensor::~LogicalCameraSensor()
{
}

//////////////////////////////////////////////////
bool LogicalCameraSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool LogicalCameraSensor::Load(sdf::ElementPtr _sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  sdf::ElementPtr cameraSdf;
  if (_sdf->GetName() == "sensor")
  {
    if (!_sdf->HasElement("logical_camera"))
    {
      ignerr << "<sensor><camera> SDF element not found while attempting to "
        << "load a ignition::sensors::LogicalCameraSensor\n";
      return false;
    }
    cameraSdf = _sdf->GetElement("logical_camera");
  }

  // These values are required in SDF, so no need to check for their
  // existence.
  this->dataPtr->frustum.SetNear(cameraSdf->Get<double>("near"));
  this->dataPtr->frustum.SetFar(cameraSdf->Get<double>("far"));
  this->dataPtr->frustum.SetFOV(cameraSdf->Get<double>("horizontal_fov"));
  this->dataPtr->frustum.SetAspectRatio(
      cameraSdf->Get<double>("aspect_ratio"));

  if (!Sensor::Load(_sdf))
    return false;

  if (this->Topic().empty())
    this->SetTopic("/logical_camera");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::LogicalCameraImage>(
      this->Topic());

  if (!this->dataPtr->pub)
  {
    ignerr << "Unable to create publisher on topic[" << this->Topic() << "].\n";
    return false;
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
void LogicalCameraSensor::SetModelPoses(
    std::map<std::string, math::Pose3d> &&_models)
{
  this->dataPtr->models = std::move(_models);
}

//////////////////////////////////////////////////
bool LogicalCameraSensor::Update(
  const std::chrono::steady_clock::duration &_now)
{
  IGN_PROFILE("LogicalCameraSensor::Update");
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // set sensor pose
  msgs::Set(this->dataPtr->msg.mutable_pose(), this->Pose());

  // set frustum pose
  this->dataPtr->frustum.SetPose(this->Pose());

  this->dataPtr->msg.clear_model();
  for (const auto &it : this->dataPtr->models)
  {
    if (this->dataPtr->frustum.Contains(it.second.Pos()))
    {
      msgs::LogicalCameraImage::Model *modelMsg =
          this->dataPtr->msg.add_model();
      modelMsg->set_name(it.first);
      msgs::Set(modelMsg->mutable_pose(), it.second - this->Pose());
    }
  }
  *this->dataPtr->msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  // Remove 'data' entries before adding new ones
  this->dataPtr->msg.mutable_header()->clear_data();
  auto frame = this->dataPtr->msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  // publish
  this->AddSequence(this->dataPtr->msg.mutable_header());
  this->dataPtr->pub.Publish(this->dataPtr->msg);

  return true;
}

//////////////////////////////////////////////////
double LogicalCameraSensor::Near() const
{
  return this->dataPtr->frustum.Near();
}

//////////////////////////////////////////////////
double LogicalCameraSensor::Far() const
{
  return this->dataPtr->frustum.Far();
}

//////////////////////////////////////////////////
ignition::math::Angle LogicalCameraSensor::HorizontalFOV() const
{
  return this->dataPtr->frustum.FOV();
}

//////////////////////////////////////////////////
double LogicalCameraSensor::AspectRatio() const
{
  return this->dataPtr->frustum.AspectRatio();
}

//////////////////////////////////////////////////
msgs::LogicalCameraImage LogicalCameraSensor::Image() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->msg;
}

IGNITION_ADD_PLUGIN(
    ignition::sensors::SensorTypePlugin<LogicalCameraSensor>,
    ignition::sensors::SensorPlugin)
IGNITION_ADD_PLUGIN_ALIAS(
    ignition::sensors::SensorTypePlugin<LogicalCameraSensor>,
    "logical_camera")
