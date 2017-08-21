#include "CameraSensorPlugin.hh"

#include <ignition/common/PluginMacros.hh>
#include <ignition/math/Angle.hh>
#include <ignition/msgs.hh>


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
bool CameraSensorPlugin::PopulateFromSDF(sdf::ElementPtr _sdf)
{
  if (_sdf->GetName() != "camera")
    return false;

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
void CameraSensorPlugin::Init(Manager *_mgr, Sensor *_sensor)
{
  this->manager = _mgr;
  this->sensor = _sensor;
}

//////////////////////////////////////////////////
bool CameraSensorPlugin::Load(sdf::ElementPtr _sdf)
{
  if (!this->PopulateFromSDF(_sdf))
    return false;

  this->pub = node.Advertise<ignition::msgs::ImageStamped>(
      this->sensor->Topic());
  if (!pub)
    return false;

  // TODO create rendering scene

  initialized = true;
  return true;
}

//////////////////////////////////////////////////
void CameraSensorPlugin::Update(const common::Time &_now)
{
  // TODO generate sensor data
  ignition::msgs::ImageStamped msg;
  this->pub.Publish(msg);
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(
    ignition::sensors::CameraSensorPlugin,
    ignition::sensors::SensorPlugin);
