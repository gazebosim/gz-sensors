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
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>
#include "SensorTailPool.hh"

using gz::sensors::SensorTailPool;

/////////////////////////////////////////////////
TEST(SensorTailPool, RunsAllJobs)
{
  auto &pool = SensorTailPool::Instance();
  std::atomic<int> counter{0};
  const int kSensors = 4, kPer = 50;
  for (int s = 0; s < kSensors; ++s)
    for (int j = 0; j < kPer; ++j)
      pool.Submit(static_cast<std::uint64_t>(1000 + s),
                  [&counter]() { counter.fetch_add(1); });
  for (int s = 0; s < kSensors; ++s)
    pool.DrainSensor(static_cast<std::uint64_t>(1000 + s));
  EXPECT_EQ(counter.load(), kSensors * kPer);
}

/////////////////////////////////////////////////
TEST(SensorTailPool, PerSensorOrderingPreserved)
{
  auto &pool = SensorTailPool::Instance();
  const std::uint64_t id = 2000;
  std::mutex m;
  std::vector<int> order;
  const int kN = 200;
  for (int i = 0; i < kN; ++i)
    pool.Submit(id, [&, i]() {
      std::lock_guard<std::mutex> lk(m);
      order.push_back(i);
    });
  pool.DrainSensor(id);
  ASSERT_EQ(static_cast<int>(order.size()), kN);
  for (int i = 0; i < kN; ++i)
    EXPECT_EQ(order[i], i) << "out of order at " << i;
}

/////////////////////////////////////////////////
TEST(SensorTailPool, SingleInFlightPerSensor)
{
  auto &pool = SensorTailPool::Instance();
  const std::uint64_t id = 3000;
  std::atomic<int> concurrent{0};
  std::atomic<int> maxConcurrent{0};
  for (int i = 0; i < 100; ++i)
    pool.Submit(id, [&]() {
      int c = concurrent.fetch_add(1) + 1;
      int prev = maxConcurrent.load();
      while (c > prev && !maxConcurrent.compare_exchange_weak(prev, c)) {}
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      concurrent.fetch_sub(1);
    });
  pool.DrainSensor(id);
  EXPECT_EQ(maxConcurrent.load(), 1) << "two jobs for one sensor ran at once";
}

/////////////////////////////////////////////////
TEST(SensorTailPool, DifferentSensorsRunConcurrently)
{
  auto &pool = SensorTailPool::Instance();
  if (pool.WorkerCount() < 2)
    GTEST_SKIP() << "needs >=2 workers";
  std::atomic<int> inA{0}, inB{0};
  std::atomic<bool> sawBoth{false};
  // Each job spins briefly checking whether the other sensor's job overlaps.
  auto mk = [&](std::atomic<int> &self, std::atomic<int> &other) {
    return [&]() {
      self.fetch_add(1);
      for (int k = 0; k < 1000 && !sawBoth.load(); ++k) {
        if (other.load() > 0) { sawBoth.store(true); break; }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
      }
      self.fetch_sub(1);
    };
  };
  pool.Submit(4001, mk(inA, inB));
  pool.Submit(4002, mk(inB, inA));
  pool.DrainSensor(4001);
  pool.DrainSensor(4002);
  EXPECT_TRUE(sawBoth.load()) << "two sensors never overlapped";
}

/////////////////////////////////////////////////
TEST(SensorTailPool, DrainWaitsForCompletion)
{
  auto &pool = SensorTailPool::Instance();
  const std::uint64_t id = 5000;
  std::atomic<bool> done{false};
  pool.Submit(id, [&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    done.store(true);
  });
  pool.DrainSensor(id);
  EXPECT_TRUE(done.load()) << "DrainSensor returned before the job finished";
}

/////////////////////////////////////////////////
TEST(SensorTailPool, ExceptionInJobDoesNotKillWorker)
{
  auto &pool = SensorTailPool::Instance();
  const std::uint64_t id = 6000;
  std::atomic<int> after{0};
  pool.Submit(id, []() { throw std::runtime_error("boom"); });
  pool.Submit(id, [&]() { after.fetch_add(1); });
  pool.DrainSensor(id);
  EXPECT_EQ(after.load(), 1) << "worker died on a throwing job";
}
