#ifndef GZ_SENSORS_SENSORTAILPOOL_HH_
#define GZ_SENSORS_SENSORTAILPOOL_HH_

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "gz/sensors/Export.hh"

namespace gz
{
namespace sensors
{
/// \brief Process-wide bounded worker pool for off-thread sensor "tail" work
/// (message build + publish). Jobs submitted under the same sensor id run
/// strictly one-at-a-time in submission order (per-sensor serialization);
/// jobs for different sensor ids run concurrently up to the worker count.
/// This serialization is what makes a sensor's unguarded per-sensor state
/// (e.g. Sensor::AddSequence's sequence map) safe to touch from a worker.
class GZ_SENSORS_VISIBLE SensorTailPool
{
  /// \brief Access the lazily-constructed process-wide pool.
  public: static SensorTailPool &Instance();

  /// \brief Submit a job for a sensor. Thread-safe.
  /// \param[in] _sensorId per-sensor key (the sensor's Id()).
  /// \param[in] _job work to run on a worker thread.
  public: void Submit(std::uint64_t _sensorId, std::function<void()> _job);

  /// \brief Block until every job for _sensorId has finished running.
  /// Call before freeing buffers a worker might still touch.
  public: void DrainSensor(std::uint64_t _sensorId);

  /// \brief Worker thread count (GZ_SENSORS_OFFTHREAD_WORKERS, default 3, min 1).
  public: unsigned int WorkerCount() const;

  public: SensorTailPool(const SensorTailPool &) = delete;
  public: SensorTailPool &operator=(const SensorTailPool &) = delete;

  private: SensorTailPool();
  private: ~SensorTailPool();
  private: void WorkerLoop();

  /// \brief Per-sensor queue and run state.
  private: struct SensorQueue
  {
    std::deque<std::function<void()>> jobs;
    bool running = false;        // a worker is executing this sensor right now
    std::uint64_t inFlight = 0;  // queued + running (for DrainSensor)
  };

  private: mutable std::mutex mutex;
  private: std::condition_variable workCv;   // wakes idle workers
  private: std::condition_variable drainCv;  // wakes DrainSensor waiters
  private: std::unordered_map<std::uint64_t, SensorQueue> queues;
  private: std::deque<std::uint64_t> ready;  // sensors with a runnable job
  private: std::vector<std::thread> workers;
  private: bool stop = false;
  private: unsigned int workerCount = 3u;
};
}  // namespace sensors
}  // namespace gz
#endif  // GZ_SENSORS_SENSORTAILPOOL_HH_
