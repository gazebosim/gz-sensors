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

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Frustum.hh>
#include <gz/math/Helpers.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/LogicalCameraSensor.hh"

using namespace gz;
using namespace sensors;

/// \brief Private data for LogicalCameraSensor
class gz::sensors::LogicalCameraSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief node to create publisher for frustum
  public: transport::Node nodeLogic;

  /// \brief publisher to publish logical camera messages.
  public: transport::Node::Publisher pub;

  /// \brief Publisher to publish logical camera frustum information
  public: transport::Node::Publisher pubLogic;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief Camera frustum.
  public: math::Frustum frustum;

  /// \brief Set world pose.
  public: math::Pose3d worldPose;

  /// \brief List of models in the world
  public: std::map<std::string, math::Pose3d> models;

  /// \brief Msg containg info on models detected by logical camera
  public: msgs::LogicalCameraImage msg;

  /// \brief Msg containing logical camera frustum info
  public: msgs::LogicalCameraSensor msgLogic;
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
      gzerr << "<sensor><camera> SDF element not found while attempting to "
        << "load a LogicalCameraSensor\n";
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
      this->dataPtr->node.Advertise<msgs::LogicalCameraImage>(
      this->Topic());

  this->dataPtr->pubLogic =
      this->dataPtr->nodeLogic.Advertise<msgs::LogicalCameraSensor>(
      this->Topic() + "/frustum");

  if (!this->dataPtr->pub)
  {
    gzerr << "Unable to create publisher on topic[" << this->Topic() << "].\n";
    return false;
  }

  if (!this->dataPtr->pubLogic)
  {
    gzerr << "Unable to create publisher on topic[" << this->Topic()
          << "/frustum].\n";
    return false;
  }

  gzdbg << "Logical images for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

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
  GZ_PROFILE("LogicalCameraSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
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
      msgs::Set(modelMsg->mutable_pose(), this->Pose().Inverse() * it.second);
    }
  }
  *this->dataPtr->msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  // Remove 'data' entries before adding new ones
  this->dataPtr->msg.mutable_header()->clear_data();
  auto frame = this->dataPtr->msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->FrameId());

  *this->dataPtr->msgLogic.mutable_header()->mutable_stamp() =
    msgs::Convert(_now);
  this->dataPtr->msgLogic.mutable_header()->clear_data();
  auto frame_log = this->dataPtr->msgLogic.mutable_header()->add_data();

  frame_log->set_key("frame_id");
  frame_log->add_value(this->FrameId());

  // publish
  this->dataPtr->msgLogic.set_near_clip(this->dataPtr->frustum.Near());
  this->dataPtr->msgLogic.set_far_clip(this->dataPtr->frustum.Far());
  this->dataPtr->msgLogic.set_horizontal_fov(
    this->dataPtr->frustum.FOV().Radian());
  this->dataPtr->msgLogic.set_aspect_ratio(
    this->dataPtr->frustum.AspectRatio());
  this->AddSequence(this->dataPtr->msg.mutable_header());

  this->dataPtr->pub.Publish(this->dataPtr->msg);
  this->dataPtr->pubLogic.Publish(this->dataPtr->msgLogic);

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
math::Angle LogicalCameraSensor::HorizontalFOV() const
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

//////////////////////////////////////////////////
bool LogicalCameraSensor::HasConnections() const
{
  return this->HasImageConnections() || this->HasFrustumConnections();
}

//////////////////////////////////////////////////
bool LogicalCameraSensor::HasImageConnections() const
{
  return this->dataPtr->pub && this->dataPtr->pub.HasConnections();
}

//////////////////////////////////////////////////
bool LogicalCameraSensor::HasFrustumConnections() const
{
  return this->dataPtr->pubLogic && this->dataPtr->pubLogic.HasConnections();
}
