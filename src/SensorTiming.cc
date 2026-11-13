/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "SensorTimingPrivate.hh"

#include <atomic>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gz/utils/NeverDestroyed.hh>

#include "gz/sensors/Sensor.hh"

namespace
{
/// \brief Whether sensor updates are being timed.
std::atomic<bool> timingEnabled{false};

/// \brief Timing of the current interval.
struct TimingStore
{
  /// \brief Protects timings. Sensors are updated from several threads.
  std::mutex mutex;

  /// \brief Timing of each sensor. Sensors are only used as a key here, they
  /// are never dereferenced because they may be gone by the time the interval
  /// is read.
  std::unordered_map<const gz::sensors::Sensor *,
    gz::sensors::SensorTiming> timings;

  /// \brief Rendering sensors that exist. Unlike the timings, it is kept
  /// from one interval to the next.
  std::unordered_set<const gz::sensors::Sensor *> renderingSensors;
};

/// \brief Renders of rendering sensors done by each thread
thread_local uint64_t threadRenders{0};

/// \brief Get the singleton storage. It is never destroyed because sensors
/// can still be updated while the process is shutting down.
/// \return The storage
TimingStore &timingStore()
{
  static gz::utils::NeverDestroyed<TimingStore> store;
  return store.Access();
}

/// \brief Get the timing of a sensor in the current interval, which starts
/// when the sensor is first seen in the interval. The store must be locked.
/// \param[in] _store The storage
/// \param[in] _sensor The sensor, which must exist
/// \return Its timing
gz::sensors::SensorTiming &sensorTiming(TimingStore &_store,
    const gz::sensors::Sensor &_sensor)
{
  auto [it, inserted] = _store.timings.try_emplace(&_sensor);
  if (inserted)
  {
    it->second.name = _sensor.Name();
    it->second.parent = _sensor.Parent();
    it->second.topic = _sensor.Topic();
    it->second.rendering = _store.renderingSensors.count(&_sensor) > 0u;
  }
  return it->second;
}
}

namespace gz
{
namespace sensors
{
inline namespace GZ_SENSORS_VERSION_NAMESPACE {

//////////////////////////////////////////////////
void SetSensorTimingEnabled(bool _enabled)
{
  timingEnabled.store(_enabled, std::memory_order_relaxed);
  if (!_enabled)
  {
    TakeSensorTimings();
  }
}

//////////////////////////////////////////////////
bool SensorTimingEnabled()
{
  return timingEnabled.load(std::memory_order_relaxed);
}

//////////////////////////////////////////////////
std::vector<SensorTiming> TakeSensorTimings()
{
  std::unordered_map<const Sensor *, SensorTiming> taken;
  {
    auto &store = timingStore();
    std::lock_guard<std::mutex> lock(store.mutex);
    taken.swap(store.timings);
  }

  std::vector<SensorTiming> result;
  result.reserve(taken.size());
  for (auto &[sensor, timing] : taken)
  {
    result.push_back(std::move(timing));
  }
  return result;
}

//////////////////////////////////////////////////
bool SensorHasConnections(const Sensor &_sensor)
{
  const bool connected = _sensor.HasConnections();
  if (SensorTimingEnabled())
  {
    auto &store = timingStore();
    std::lock_guard<std::mutex> lock(store.mutex);
    sensorTiming(store, _sensor).subscribers =
        connected ? SensorSubscribers::SOME : SensorSubscribers::NONE;
  }
  return connected;
}

//////////////////////////////////////////////////
void RecordSensorTiming(const Sensor &_sensor,
    std::chrono::nanoseconds _total, std::chrono::nanoseconds _gpuWait,
    bool _rendered)
{
  auto &store = timingStore();
  std::lock_guard<std::mutex> lock(store.mutex);
  auto &timing = sensorTiming(store, _sensor);
  ++timing.updates;
  timing.total += _total;
  timing.gpuWait += _gpuWait;
  if (_rendered)
    ++timing.renders;
}

//////////////////////////////////////////////////
uint64_t ThreadRenderCount()
{
  return threadRenders;
}

//////////////////////////////////////////////////
void CountThreadRender()
{
  ++threadRenders;
}

//////////////////////////////////////////////////
void AddRenderingSensor(const Sensor &_sensor)
{
  auto &store = timingStore();
  std::lock_guard<std::mutex> lock(store.mutex);
  store.renderingSensors.insert(&_sensor);
}

//////////////////////////////////////////////////
void RemoveRenderingSensor(const Sensor &_sensor)
{
  auto &store = timingStore();
  std::lock_guard<std::mutex> lock(store.mutex);
  store.renderingSensors.erase(&_sensor);
}

}
}
}
