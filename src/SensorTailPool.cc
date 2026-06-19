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
  static SensorTailPool pool;
  return pool;
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
