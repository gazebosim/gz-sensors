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

#ifndef GZ_SENSORS_PUBLISHINPLACE_HH_
#define GZ_SENSORS_PUBLISHINPLACE_HH_

#include <google/protobuf/io/coded_stream.h>
#include <gz/msgs/pointcloud_packed.pb.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>

#include <gz/transport/Node.hh>

// Internal helpers to publish a message whose large bytes field is written
// once, straight into the transport's buffer (not installed).
namespace gz
{
namespace sensors
{
/// \brief Write the key of the length-delimited field number _field of
/// _size bytes: its tag and its varint length, at most 15 bytes.
/// \return The end of the key in _out
inline uint8_t *WriteLengthDelimitedKey(int _field, std::size_t _size,
    uint8_t *_out)
{
  using google::protobuf::io::CodedOutputStream;
  _out = CodedOutputStream::WriteTagToArray(
      (static_cast<uint32_t>(_field) << 3) | 2, _out);
  return CodedOutputStream::WriteVarint64ToArray(_size, _out);
}

/// \brief Publish a message in place: _front and _back are its serialized
/// fields before and after the bytes field number _field, whose _size bytes
/// _fill writes straight into the transport's buffer (false cancels the
/// message). Protobuf serializes fields in field number order and omits an
/// empty bytes field, so the bytes are those Publish() sends for the whole
/// message.
/// \return The result of PublishInPlace
inline bool PublishWithData(transport::Node::Publisher &_pub,
    const std::string &_front, int _field, std::size_t _size,
    const std::string &_back, const std::function<bool(char *)> &_fill)
{
  uint8_t key[15];
  const std::size_t keySize =
      _size > 0 ? WriteLengthDelimitedKey(_field, _size, key) - key : 0;
  return _pub.PublishInPlace(
      _front.size() + keySize + _size + _back.size(), [&](char *_buffer)
      {
        memcpy(_buffer, _front.data(), _front.size());
        _buffer += _front.size();
        memcpy(_buffer, key, keySize);
        _buffer += keySize;
        if (!_fill(_buffer))
          return false;
        memcpy(_buffer + _size, _back.data(), _back.size());
        return true;
      });
}

/// \brief Publish a msgs::PointCloudPacked in place: _front is its
/// serialized fields before the data, whose _size bytes _fill writes,
/// returning whether the cloud is dense. is_dense (after the data) follows
/// the data, but the message size depends on it: the message is serialized
/// with _dense, the last cloud's, and only if _fill finds otherwise it is
/// written again. This assumes an unthrottled publisher: PublishInPlace
/// takes the throttling slot before it calls the fill, so with MsgsPerSec
/// the second attempt would be dropped.
/// \param[in,out] _dense The last cloud's is_dense; this one's on return,
/// unchanged if the message was not written
inline void PublishPointCloudWithData(transport::Node::Publisher &_pub,
    const std::string &_front, std::size_t _size,
    const std::function<bool(char *)> &_fill, bool &_dense)
{
  for (int attempt = 0; attempt < 2; ++attempt)
  {
    msgs::PointCloudPacked tail;
    tail.set_is_dense(_dense);
    bool filled = false;
    bool dense = _dense;
    PublishWithData(_pub, _front, msgs::PointCloudPacked::kDataFieldNumber,
        _size, tail.SerializeAsString(), [&](char *_out)
        {
          dense = _fill(_out);
          filled = true;
          return dense == _dense;
        });
    if (!filled || dense == _dense)
      return;
    _dense = dense;
  }
}
}  // namespace sensors
}  // namespace gz

#endif
