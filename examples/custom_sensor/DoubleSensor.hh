/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef DOUBLESENSOR_HH_
#define DOUBLESENSOR_HH_

#include <ignition/sensors/Sensor.hh>
#include <ignition/sensors/SensorTypes.hh>
#include <ignition/transport/Node.hh>

namespace custom
{
  /// \brief Example sensor that publishes a noisy double value.
  class DoubleSensor : public ignition::sensors::Sensor
  {
    /// \brief Load the sensor with SDF parameters.
    /// \param[in] _sdf SDF Sensor parameters.
    /// \return True if loading was successful
    public: virtual bool Load(const sdf::Sensor &_sdf) override;

    /// \brief Update the sensor and generate data
    /// \param[in] _now The current time
    /// \return True if the update was successfull
    public: virtual bool Update(
      const std::chrono::steady_clock::duration &_now) override;

    /// \brief Latest data
    public: double data{0.0};

    /// \brief Noise that will be applied to the sensor data
    private: ignition::sensors::NoisePtr noise{nullptr};

    /// \brief Node for communication
    private: ignition::transport::Node node;

    /// \brief Publishes sensor data
    private: ignition::transport::Node::Publisher pub;
  };
}

#endif
