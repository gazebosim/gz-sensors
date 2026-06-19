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
#include "SensorTailPool.hh"

#include <cstdlib>
#include <exception>
#include <utility>

#include <gz/common/Console.hh>

using namespace gz;
using namespace sensors;

namespace {
unsigned int ReadWorkerCount()
{
  const char *env = std::getenv("GZ_SENSORS_OFFTHREAD_WORKERS");
  if (!env || env[0] == '\0')
    return 3u;
  const int v = std::atoi(env);
  return v < 1 ? 1u : static_cast<unsigned int>(v);
}
}  // namespace

//////////////////////////////////////////////////
SensorTailPool &SensorTailPool::Instance()
{
  // Deliberately heap-allocated and never deleted. This pool is process-wide
  // and must outlive every sensor that can submit to it. A function-local
  // static *object* would be destroyed at static-destruction time; a sensor
  // kept alive until then (e.g. in an embedder's global) would then call
  // Submit()/DrainSensor() on a destroyed pool. Leaking removes that ordering
  // hazard entirely. The OS reclaims the memory and the idle worker threads at
  // process exit; every well-behaved sensor drains its own in-flight jobs in
  // its destructor before then, so no worker ever touches a dead sensor.
  static SensorTailPool *const instance = new SensorTailPool();
  return *instance;
}

//////////////////////////////////////////////////
SensorTailPool::SensorTailPool()
{
  this->workerCount = ReadWorkerCount();
  for (unsigned int i = 0; i < this->workerCount; ++i)
    this->workers.emplace_back([this]() { this->WorkerLoop(); });
}

//////////////////////////////////////////////////
SensorTailPool::~SensorTailPool()
{
  // Not reached for the process-wide Instance() (intentionally leaked); retained
  // for completeness and any future explicitly-owned pool.
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->stop = true;
  }
  this->workCv.notify_all();
  for (auto &t : this->workers)
  {
    if (t.joinable())
      t.join();
  }
}

//////////////////////////////////////////////////
unsigned int SensorTailPool::WorkerCount() const
{
  return this->workerCount;
}

//////////////////////////////////////////////////
void SensorTailPool::Submit(std::uint64_t _sensorId, std::function<void()> _job)
{
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    SensorQueue &q = this->queues[_sensorId];
    q.jobs.push_back(std::move(_job));
    ++q.inFlight;
    // Becomes runnable (and is not already in `ready` nor running) exactly when
    // it transitions from 0 queued jobs while not running.
    if (!q.running && q.jobs.size() == 1u)
      this->ready.push_back(_sensorId);
  }
  this->workCv.notify_one();
}

//////////////////////////////////////////////////
void SensorTailPool::DrainSensor(std::uint64_t _sensorId)
{
  std::unique_lock<std::mutex> lock(this->mutex);
  this->drainCv.wait(lock, [this, _sensorId]() {
    auto it = this->queues.find(_sensorId);
    return it == this->queues.end() || it->second.inFlight == 0;
  });
}

//////////////////////////////////////////////////
void SensorTailPool::WorkerLoop()
{
  std::unique_lock<std::mutex> lock(this->mutex);
  while (true)
  {
    this->workCv.wait(lock, [this]() {
      return this->stop || !this->ready.empty();
    });
    if (this->stop && this->ready.empty())
      return;

    const std::uint64_t id = this->ready.front();
    this->ready.pop_front();
    SensorQueue &q = this->queues[id];
    q.running = true;
    std::function<void()> job = std::move(q.jobs.front());
    q.jobs.pop_front();

    lock.unlock();
    try
    {
      job();
    }
    catch (const std::exception &e)
    {
      gzerr << "Sensor tail job threw: " << e.what() << std::endl;
    }
    catch (...)
    {
      gzerr << "Sensor tail job threw an unknown exception" << std::endl;
    }
    lock.lock();

    SensorQueue &q2 = this->queues[id];
    q2.running = false;
    --q2.inFlight;
    if (!q2.jobs.empty())
      this->ready.push_back(id);
    if (q2.inFlight == 0)
      this->drainCv.notify_all();
  }
}
