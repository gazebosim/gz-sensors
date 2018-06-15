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
#ifndef IGNITION_SENSORS_GPULIDARSENSOR_HH_
#define IGNITION_SENSORS_GPULIDARSENSOR_HH_

#include <string>
#include <vector>
#include <memory>
#include <sstream>

#include <sdf/sdf.hh>

#include <ignition/common/Time.hh>
#include <ignition/common/Timer.hh>
#include <ignition/common/Event.hh>
#include <ignition/common/Console.hh>
//#include <ignition/common/Exception.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/PluginMacros.hh>

#include <ignition/sensors/Sensor.hh>
#include <ignition/sensors/Export.hh>
#include <ignition/sensors/LidarSensor.hh>
//#include <ignition/sensors/ign_sensors_gpu_lidar_export.hh>

#include <ignition/msgs.hh>

#include <ignition/math/Color.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Angle.hh>

#include <ignition/rendering/ogre/OgreIncludes.hh>
#include <ignition/rendering/ogre/OgreIncludes.hh>
#include <ignition/rendering/ogre/OgreObject.hh>
#include <ignition/rendering/ogre/OgreConversions.hh>
#include <ignition/rendering/ogre/Export.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/RenderTypes.hh>

/*
#ifndef _WIN32
  #include <dirent.h>
#else
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
  #include "gazebo/common/win_dirent.h"
#endif
*/

namespace Ogre
{
  class Material;
  class Renderable;
  class Pass;
  class AutoParamDataSource;
  class Matrix4;
  class MovableObject;
}

namespace ignition
{
  namespace sensors
  {
    /// \brief forward declarations
    class GpuLidarSensorPrivate;

    /// \brief GpuLidar Sensor Class
    ///
    ///   This class creates laser scans using the GPU. It's measures the range
    ///   from the origin of the center to points on the visual geometry in the
    ///   scene.
    ///
    ///   It offers both an ignition-transport interface and a direct C++ API
    ///   to access the image data. The API works by setting a callback to be
    ///   called with image data.
    class IGNITION_SENSORS_VISIBLE GpuLidarSensor : public LidarSensor
    {
      /// \brief constructor
      public: GpuLidarSensor();

      /// \brief destructor
      public: virtual ~GpuLidarSensor();

      /// \brief Horizontal half angle.
      protected: double horzHalfAngle;

      /// \brief Vertical half angle.
      protected: double vertHalfAngle;

      /// \brief Ray count ratio.
      protected: double rayCountRatio;

      /// \brief Horizontal field-of-view.
      protected: double hfov;

      /// \brief Vertical field-of-view.
      protected: double vfov;

      /// \brief Cos horizontal field-of-view.
      protected: double chfov;

      /// \brief Cos vertical field-of-view.
      protected: double cvfov;

      /// \brief Near clip plane.
      protected: double nearClip;

      /// \brief Far clip plane.
      protected: double farClip;

      /// \brief True if the sensor is horizontal only.
      protected: bool isHorizontal;

      /// \brief Number of cameras needed to generate the rays.
      protected: unsigned int cameraCount;

      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<GpuLidarSensorPrivate> dataPtr;

      /// \brief Force the sensor to generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(const common::Time &_now) override;

      /// \brief Initialize values in the sensor
      /// \return True on success
      public: virtual bool Init() override;

      // Documentation inherited
      public: virtual void Fini();

      /// brief Render the camera.
      private: void Render();

      /// \brief Create the texture which is used to render laser data.
      /// \param[in] _textureName Name of the new texture.
      public: void CreateLaserTexture(const std::string &_textureName);

      // Documentation inherited
      public: virtual void PostRender();

      /// \brief Set the number of laser samples in the width and height
      /// \param[in] _w Number of samples in the horizontal sweep
      /// \param[in] _h Number of samples in the vertical sweep
      public: void SetRangeCount(const unsigned int _w,
          const unsigned int _h = 1);

      /// \brief Get (horizontal_max_angle + horizontal_min_angle) * 0.5
      /// \return (horizontal_max_angle + horizontal_min_angle) * 0.5
      public: double HorzHalfAngle() const;

      /// \brief Get (vertical_max_angle + vertical_min_angle) * 0.5
      /// \return (vertical_max_angle + vertical_min_angle) * 0.5
      public: double VertHalfAngle() const;

      /// \brief Set the horizontal half angle
      /// \param[in] _angle horizontal half angle
      public: void SetHorzHalfAngle(const double _angle);

      /// \brief Set the vertical half angle
      /// \param[in] _angle vertical half angle
      public: void SetVertHalfAngle(const double _angle);

      /// \brief Set sensor horizontal or vertical
      /// \param[in] _horizontal True if horizontal, false if not
      public: void SetIsHorizontal(const bool _horizontal);

      /// \brief Gets if sensor is horizontal
      /// \return True if horizontal, false if not
      public: bool IsHorizontal() const;

      /// \brief Get the horizontal field of view of the laser sensor.
      /// \return The horizontal field of view of the laser sensor.
      public: double HorzFOV() const;

      /// \brief Get Cos Horz field-of-view
      /// \return 2 * atan(tan(this->hfov/2) / cos(this->vfov/2))
      public: double CosHorzFOV() const;

      /// \brief Set the Cos Horz FOV
      /// \param[in] _chfov Cos Horz FOV
      public: void SetCosHorzFOV(const double _chfov);

      /// \brief Get the vertical field-of-view.
      /// \return The vertical field of view of the laser sensor.
      public: double VertFOV() const;

      /// \brief Get Cos Vert field-of-view
      /// \return 2 * atan(tan(this->vfov/2) / cos(this->hfov/2))
      public: double CosVertFOV() const;

      /// \brief Set the Cos Horz FOV
      /// \param[in] _cvfov Cos Horz FOV
      public: void SetCosVertFOV(const double _cvfov);

      /// \brief Get near clip
      /// \return near clip distance
      public: double NearClip() const;

      /// \brief Get far clip
      /// \return far clip distance
      public: double FarClip() const;

      /// \brief Set the near clip distance
      /// \param[in] _near near clip distance
      public: void SetNearClip(const double _near);

      /// \brief Set the far clip distance
      /// \param[in] _far far clip distance
      public: void SetFarClip(const double _far);

      /// \brief Set the horizontal fov
      /// \param[in] _hfov horizontal fov
      public: void SetHorzFOV(const double _hfov);

      /// \brief Set the vertical fov
      /// \param[in] _vfov vertical fov
      public: void SetVertFOV(const double _vfov);

      /// \brief Get the number of cameras required
      /// \return Number of cameras needed to generate the rays
      public: unsigned int CameraCount() const;

      /// \brief Set the number of cameras required
      /// \param[in] _cameraCount The number of cameras required to generate
      /// the rays
      public: void SetCameraCount(const unsigned int _cameraCount);

      /// \brief Get the ray count ratio (equivalent to aspect ratio)
      /// \return The ray count ratio (equivalent to aspect ratio)
      public: double RayCountRatio() const;

      /// \brief Sets the ray count ratio (equivalen to aspect ratio)
      /// \param[in] _rayCountRatio ray count ratio (equivalent to aspect ratio)
      public: void SetRayCountRatio(const double _rayCountRatio);

      // Documentation inherited.
      private: virtual void RenderImpl();

      /// \brief Update a render target.
      /// \param[in, out] _target Render target to update (render).
      /// \param[in, out] _material Material used during render.
      /// \param[in] _cam Camerat to render from.
      /// \param[in] _updateTex True to update the textures in the material
      private: void UpdateRenderTarget(Ogre::RenderTarget *_target,
                                       Ogre::Material *_material,
                                       Ogre::Camera *_cam,
                                       const bool _updateTex = false);

      /// \internal
      /// \brief Implementation of Ogre::RenderObjectListener
      public: virtual void notifyRenderSingleObject(Ogre::Renderable *_rend,
              const Ogre::Pass *_p, const Ogre::AutoParamDataSource *_s,
              const Ogre::LightList *_ll, bool _supp);

      /// \brief Create an ortho camera.
      private: void CreateOrthoCam();

      /// \brief Create a mesh.
      private: void CreateMesh();

      /// \brief Create a canvas.
      private: void CreateCanvas();

      /// \brief Builds scaled Orthogonal Matrix from parameters.
      /// \param[in] _left Left clip.
      /// \param[in] _right Right clip.
      /// \param[in] _bottom Bottom clip.
      /// \param[in] _top Top clip.
      /// \param[in] _near Near clip.
      /// \param[in] _far Far clip.
      /// \return The Scaled orthogonal Ogre::Matrix4
      private: Ogre::Matrix4 BuildScaledOrthoMatrix(const float _left,
          const float _right, const float _bottom, const float _top,
          const float _near, const float _far);

      /// \brief Sets first pass target.
      /// \param[in] _target Render target for the first pass.
      /// \param[in] _index Index of the texture.
      private: virtual void Set1stPassTarget(Ogre::RenderTarget *_target,
                                             const unsigned int _index);

      /// \brief Sets second pass target.
      /// \param[in] _target Render target for the second pass.
      private: virtual void Set2ndPassTarget(Ogre::RenderTarget *_target);


    };
  }
}

#endif

