/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef IGNITION_SENSORS_NOISE_HH_
#define IGNITION_SENSORS_NOISE_HH_

#include <functional>
#include <string>
#include <vector>

#include <ignition/rendering/RenderTypes.hh>
#include <ignition/sensors/config.hh>
#include <ignition/sensors/SensorTypes.hh>
#include <ignition/sensors/Export.hh>

#include <sdf/sdf.hh>

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    // Forward declarations
    class NoisePrivate;

    /// \class NoiseFactory Noise.hh ignition/sensors/Noise.hh
    /// \brief Use this noise manager for creating and loading noise models.
    class IGNITION_SENSORS_VISIBLE NoiseFactory
    {
      /// \brief Load a noise model based on the input sdf parameters and
      /// sensor type.
      /// \param[in] _sdf Noise sdf parameters.
      /// \param[in] _sensorType Type of sensor. This is currently used to
      /// distinguish between image and non image sensors in order to create
      /// the appropriate noise model.
      /// \return Pointer to the noise model created.
      public: static NoisePtr NewNoiseModel(sdf::ElementPtr _sdf,
          const std::string &_sensorType = "");
    };

    /// \brief Which noise types we support
    enum class IGNITION_SENSORS_VISIBLE NoiseType : int
    {
      NONE = 0,
      CUSTOM = 1,
      GAUSSIAN = 2
    };

    /// \class Noise Noise.hh ignition/sensors/Noise.hh
    /// \brief Noise models for sensor output signals.
    class IGNITION_SENSORS_VISIBLE Noise
    {
      /// \brief Constructor. This should not be called directly unless creating
      /// an empty noise model. Use NoiseFactory::NewNoiseModel to instantiate
      /// a new noise model.
      /// \param[in] _type Type of noise model.
      /// \sa NoiseFactory::NewNoiseModel
      public: explicit Noise(NoiseType _type);

      /// \brief Destructor.
      public: virtual ~Noise();

      /// \brief Load noise parameters from sdf.
      /// \param[in] _sdf SDF parameters.
      /// \param[in] _sensor Type of sensor.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Apply noise to input data value.
      /// \param[in] _in Input data value.
      /// \return Data with noise applied.
      public: double Apply(double _in);

      /// \brief Apply noise to input data value. This gets overriden by
      /// derived classes, and called by Apply.
      /// \param[in] _in Input data value.
      /// \return Data with noise applied.
      public: virtual double ApplyImpl(double _in);

      /// \brief Accessor for NoiseType.
      /// \return Type of noise currently in use.
      public: NoiseType Type() const;

      /// \brief Register a custom noise callback.
      /// \param[in] _cb Callback function for applying a custom noise model.
      /// This is useful if users want to use their own noise model from a
      /// sensor plugin.
      public: virtual void SetCustomNoiseCallback(
          std::function<double(double)> _cb);

      /// \brief Set camera needed to create image noise. This is only needed
      /// for image sensors, i.e. camera/multicamera/depth sensors, which use
      /// shaders for more efficient noise generation.
      /// \param[in] _camera Camera associated to an image sensor
      public: virtual void SetCamera(rendering::CameraPtr _camera);

      /// \brief Output information about the noise model.
      /// \param[in] _out Output stream
      public: virtual void Print(std::ostream &_out) const;

      /// \brief Private data pointer
      private: NoisePrivate *dataPtr = nullptr;
    };
    }
  }
}
#endif
