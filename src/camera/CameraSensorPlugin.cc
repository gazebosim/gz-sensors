#include "CameraSensorPlugin.hh"

#include <ignition/math/Angle.hh>

using namespace ignition::sensors;


//////////////////////////////////////////////////
CameraSensorPlugin::CameraSensorPlugin()
{

}


//////////////////////////////////////////////////
CameraSensorPlugin::~CameraSensorPlugin()
{

}

//////////////////////////////////////////////////
bool CameraSensorPlugin::Load(sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("camera"))
    return false;
  sdf::ElementPtr cameraElem = _sdf->GetElement("camera");

  sdf::ElementPtr imgElem = _sdf->GetElement("image");
  if (!imgElem)
    return false;

  this->imageWidth = imgElem->Get<int>("width");
  this->imageHeight = imgElem->Get<int>("height");
  // TODO Pixel format

  // Create the directory to store frames
  if (_sdf->HasElement("save") &&
      _sdf->GetElement("save")->Get<bool>("enabled"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("save");
    std::string path = elem->Get<std::string>("path");
    // TODO make directory, or delegate saving images to another class
  }

  if (_sdf->HasElement("horizontal_fov"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("horizontal_fov");
    ignition::math::Angle angle = elem->Get<double>();
    if (angle < 0.01 || angle > M_PI*2)
      return false;
    this->horizontalFieldOfView = angle.Radian();
  }

  if (_sdf->HasElement("distortion"))
  {
    // TODO Port Distortion class
    // This->dataPtr->distortion.reset(new Distortion());
    // This->dataPtr->distortion->Load(this->sdf->GetElement("distortion"));
  }

  return true;
}

//////////////////////////////////////////////////
void CameraSensorPlugin::Update(const common::Time &_now)
{
  // TODO generate sensor data
}
