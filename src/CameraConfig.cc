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

#include <ignition/sensors/CameraConfig.hh>
#include <ignition/common/Image.hh>
#include <ignition/math/Helpers.hh>

#include <random>
#include <sstream>

#include <ignition/transport/TopicUtils.hh>


using namespace ignition::sensors;

/// \brief Private class for CameraConfig
/// \internal
class ignition::sensors::CameraConfigPrivate
{
  /// \brief name of the camera
  public: std::string name;

  /// \brief topic to publish to
  public: std::string topic;

  /// \brief frames to generate per second
  public: double hz = 0;

  /// \brief width in pixels
  public: double width = 128;

  /// \brief height in pixels
  public: double height = 128;

  /// \brief Horizontal field of view in radians
  public: double hfov = 1.05;

  /// \brief Noise mean
  public: double noiseMean = 0;

  /// \brief Noise standard desviation
  public: double noiseStdDev = 0;

  /// \brief near_ clip plane distance in meters
  public: double near_ = 0.1;

  /// \brief far_ clip plane distance in meters
  public: double far_ = 100.0;

  /// \brief format used for pixel data in output images
  public: ignition::common::Image::PixelFormatType format =
          ignition::common::Image::RGB_INT8;
};


//////////////////////////////////////////////////
/// \brief Returns a string with random characters
/// \param[in] _num number of characters to generate
std::string randomString(std::size_t _num)
{
  const char * const lut = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution(0, sizeof(lut));
  std::string out(_num, 'X');
  for (std::size_t i = 0; i < _num; ++i)
  {
    out[i] = lut[distribution(generator)];
  }
  return out;
}

//////////////////////////////////////////////////
CameraConfig::CameraConfig(const std::string &_name,
        const std::string &_topic, double _hz, std::size_t _width,
        std::size_t _height, double _hfov, double _near, double _far,
        ignition::common::Image::PixelFormatType _format)
  : dataPtr(new CameraConfigPrivate)
{
  if (!this->SetName(_name))
  {
    this->SetName(std::string("camera") + randomString(7));
  }

  if (!this->SetTopic(_topic))
  {
    this->SetTopic(std::string("/sensor/camera/" + this->dataPtr->name));
  }
  this->SetHz(_hz);
  this->SetWidth(_width);
  this->SetHeight(_height);
  this->SetHorizontalFOV(_hfov);
  this->SetClip(_near, _far);
  this->SetFormat(_format);
}

//////////////////////////////////////////////////
CameraConfig::CameraConfig()
{
  // Create a random name since none was provided
  this->SetTopic(std::string("camera") + randomString(7));
  this->SetTopic(std::string("/sensor/camera/" + this->dataPtr->name));
}

//////////////////////////////////////////////////
CameraConfig::~CameraConfig()
{
}

//////////////////////////////////////////////////
bool CameraConfig::SetName(const std::string &_name)
{
  // Must be convertible to a topic in case the topic is auto-generated
  // Cannot have a leading slash because it gets appended to a string with
  // a trailing slash and that would make for an illegal topic name
  if (ignition::transport::TopicUtils::IsValidTopic(_name)
      && _name[0] != '/')
  {
    this->dataPtr->name = _name;
    return true;
  }
  return false;
}

//////////////////////////////////////////////////
const std::string &CameraConfig::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
bool CameraConfig::SetTopic(const std::string &_topic)
{
  if (!ignition::transport::TopicUtils::IsValidTopic(_topic))
    return false;

  this->dataPtr->topic = _topic;
  return true;
}

//////////////////////////////////////////////////
const std::string &CameraConfig::Topic() const
{
  return this->dataPtr->topic;
}

//////////////////////////////////////////////////
bool CameraConfig::SetHz(double _hz)
{
  if (_hz <= 0)
    return false;

  this->dataPtr->hz = _hz;
  return true;
}

//////////////////////////////////////////////////
double CameraConfig::Hz() const
{
  return this->dataPtr->hz;
}

//////////////////////////////////////////////////
void CameraConfig::SetNoiseMean(double _mean)
{
  this->dataPtr->noiseMean = _mean;
}

//////////////////////////////////////////////////
double CameraConfig::NoiseMean() const
{
  return this->dataPtr->noiseStdDev;
}

//////////////////////////////////////////////////
bool CameraConfig::SetNoiseStdDev(double _stddev)
{
  if (_stddev < 0)
    return false;

  this->dataPtr->noiseStdDev = _stddev;
  return true;
}

//////////////////////////////////////////////////
double CameraConfig::NoiseStdDev() const
{
  return this->dataPtr->noiseStdDev;
}

//////////////////////////////////////////////////
bool CameraConfig::SetWidth(std::size_t _width)
{
  if (_width == 0)
    return false;

  this->dataPtr->width = _width;
  return true;
}

//////////////////////////////////////////////////
std::size_t CameraConfig::Width() const
{
  return this->dataPtr->width;
}

//////////////////////////////////////////////////
bool CameraConfig::SetHeight(std::size_t _height)
{
  if (_height == 0)
    return false;

  this->dataPtr->height = _height;
  return true;
}

//////////////////////////////////////////////////
std::size_t CameraConfig::Height() const
{
  return this->dataPtr->height;
}

//////////////////////////////////////////////////
bool CameraConfig::SetHorizontalFOV(double _hfov)
{
  if (_hfov <= 0 || _hfov > IGN_PI / 2.0)
    return false;

  this->dataPtr->hfov = _hfov;
  return true;
}

//////////////////////////////////////////////////
double CameraConfig::HorizontalFOV() const
{
  return this->dataPtr->hfov;
}

//////////////////////////////////////////////////
bool CameraConfig::SetClip(double _near, double _far)
{
  if (_near <= 0 || _far <= 0 || _far <= _near)
    return false;

  this->dataPtr->near_ = _near;
  this->dataPtr->far_ = _far;
  return true;
}

//////////////////////////////////////////////////
double CameraConfig::Near() const
{
  return this->dataPtr->near_;
}

//////////////////////////////////////////////////
double CameraConfig::Far() const
{
  return this->dataPtr->far_;
}

//////////////////////////////////////////////////
double CameraConfig::VerticalFOV() const
{
  const double &hfov = this->dataPtr->hfov;
  const double width = this->dataPtr->width;
  const double height = this->dataPtr->height;
  return height * hfov / width;
}

//////////////////////////////////////////////////
bool CameraConfig::SetFormat(ignition::common::Image::PixelFormatType _format)
{
  if (_format <= 0 || _format >= ignition::common::Image::PIXEL_FORMAT_COUNT)
    return false;

  this->dataPtr->format = _format;
  return true;
}

//////////////////////////////////////////////////
ignition::common::Image::PixelFormatType CameraConfig::Format() const
{
  return this->dataPtr->format;
}

//////////////////////////////////////////////////
sdf::ElementPtr CameraConfig::ToSDF()
{
  // \todo(nkoenig) output pixel format to SDF
  std::ostringstream stream;
  stream
    << "<?xml version='1.0'?>"
    << "<sdf version='1.6'>"
    << " <model name='m1'>"
    << "  <link name='link1'>"
    << "   <sensor name='" << this->dataPtr->name << "' type='camera'>"
    << "    <update_rate>" << this->dataPtr->hz << "</update_rate>"
    << "    <topic>" << this->dataPtr->topic << "</topic>"
    << "    <camera>"
    << "     <horizontal_fov>" << this->dataPtr->hfov << "</horizontal_fov>"
    << "     <image>"
    << "      <width>" << this->dataPtr->width << "</width>"
    << "      <height>" << this->dataPtr->height << "</height>"
    << "      <format>"
    << ignition::common::PixelFormatNames[this->dataPtr->format] << "</format>"
    << "     </image>"
    << "     <clip>"
    << "      <near>" << this->dataPtr->near_ << "</near>"
    << "      <far>" << this->dataPtr->far_ << "</far>"
    << "     </clip>"
    << "    </camera>"
    << "   </sensor>"
    << "  </link>"
    << " </model>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  if (!sdf::readString(stream.str(), sdfParsed))
    return sdf::ElementPtr();

  return sdfParsed->Root()->GetElement("model")->GetElement("link")
    ->GetElement("sensor");
}
