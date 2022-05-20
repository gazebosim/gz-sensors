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
#ifndef TEST_INTEGRATION_TRANSPORTTESTTOOLS_HH_
#define TEST_INTEGRATION_TRANSPORTTESTTOOLS_HH_

#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>

#include <gz/transport.hh>

/// \brief class which simplifies waiting for a message to be received
template <typename M>
class WaitForMessageTestHelper
{
  /// \brief Constructor
  /// \param[in] _topic name of topic to listen to
  public: explicit WaitForMessageTestHelper(const std::string &_topic)
  {
    if (this->node.Subscribe(_topic, &WaitForMessageTestHelper<M>::OnMessage,
          this))
      this->subscriptionCreated = true;
    else
      this->diagnostics = "Failed to create subscription to " + _topic;
  }

  protected: void OnMessage(const M &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    // Set condition variable
    this->gotMessage = true;
    this->msg.CopyFrom(_msg);
    this->conditionVariable.notify_all();
  }

  /// \brief waits forever for a message to be received
  /// \return true if a message was received
  /// \remarks Set CTest timeout property for control over time
  public: bool WaitForMessage()
  {
    std::unique_lock<std::mutex> lock(this->mtx);
    if (this->subscriptionCreated)
    {
      this->conditionVariable.wait(lock, [this]{return this->gotMessage;});
    }
    bool success = this->gotMessage;
    this->gotMessage = false;
    return success;
  }

  /// \brief waits for a message to be received with a timeout
  /// \return true if a message was received
  /// \param[in] _timeout Time to wait for a message.
  /// \remarks Set CTest timeout property for control over time
  public: bool WaitForMessage(
      const std::chrono::steady_clock::duration &_timeout)
  {
    std::unique_lock<std::mutex> lock(this->mtx);
    if (this->subscriptionCreated)
    {
      this->conditionVariable.wait_for(lock, _timeout,
          [this]{return this->gotMessage;});
    }
    bool success = this->gotMessage;
    this->gotMessage = false;
    return success;
  }

  /// \brief Get the last msg received.
  /// \return last msg received.
  public: M Message()
  {
    return this->msg;
  }

  /// \brief Node to subscribe to topics
  public: gz::transport::Node node;

  /// \brief True if subscription was created
  public: bool subscriptionCreated = false;

  /// \brief True if a message has been received
  public: bool gotMessage = false;

  /// \brief a mutex used for the condition variable
  public: std::mutex mtx;

  /// \brief the msg received on the specified topic.
  public: M msg;

  /// \brief variable used to notivy waiting thread
  public: std::condition_variable conditionVariable;

  /// \brief a string containing diagnostics if a problem happens
  public: std::string diagnostics;

  /// \brief output diagnostics
  friend std::ostream & operator<<(std::ostream &_os,
      const WaitForMessageTestHelper<M>& _helper)
  {
    return _os << _helper.diagnostics;
  }
};


#endif
