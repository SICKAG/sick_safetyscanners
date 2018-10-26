// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2018, SICK AG, Waldkirch
*  Copyright (C) 2018, FZI Forschungszentrum Informatik, Karlsruhe, Germany
*
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

*/

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!
 * \file TCPPacketMerger.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_microscan3_ros_driver/data_processing/TCPPacketMerger.h>

namespace sick {
namespace data_processing {

TCPPacketMerger::TCPPacketMerger()
  : m_is_complete(false)
  , m_deployed_packet_buffer()
{
}


bool TCPPacketMerger::isComplete() const
{
  return m_is_complete;
}

bool TCPPacketMerger::isEmpty() const
{
  return m_buffer_vector.empty();
}

sick::datastructure::PacketBuffer TCPPacketMerger::getDeployedPacketBuffer()
{
  m_is_complete = false;
  return m_deployed_packet_buffer;
}

bool TCPPacketMerger::addTCPPacket(const datastructure::PacketBuffer& buffer)
{
  if (isComplete())
  {
    m_is_complete = false;
  }

  addToMap(buffer);
  deployPacketIfComplete();
  return isComplete();
}

bool TCPPacketMerger::addToMap(const datastructure::PacketBuffer& newPacket)
{
  uint32_t currentSize   = getCurrentSize();
  uint32_t remainingSize = m_targetSize - currentSize;
  m_buffer_vector.push_back(newPacket);
  if (remainingSize == newPacket.getLength())
  {
    m_is_complete = true;
  }

  return isComplete();
}

bool TCPPacketMerger::deployPacketIfComplete()
{
  if (isComplete())
  {
    deployPacket();
    return true;
  }
  return false;
}

bool TCPPacketMerger::deployPacket()
{
  sick::datastructure::PacketBuffer::VectorBuffer headerless_packet_buffer;
  for (auto& parsed_packet_buffer : m_buffer_vector)
  {
    sick::datastructure::PacketBuffer packet_buffer = parsed_packet_buffer.getBuffer();

    headerless_packet_buffer.insert(headerless_packet_buffer.end(),
                                    packet_buffer.getBuffer().begin(),
                                    packet_buffer.getBuffer().end());
  }
  m_deployed_packet_buffer.setBuffer(headerless_packet_buffer);
  m_buffer_vector.clear();
  return true;
}

uint32_t TCPPacketMerger::getTargetSize() const
{
  return m_targetSize;
}

void TCPPacketMerger::setTargetSize(const uint32_t& targetSize)
{
  m_targetSize = targetSize;
}

uint32_t TCPPacketMerger::getCurrentSize() const
{
  size_t sum = 0;
  for (auto it_packet = m_buffer_vector.begin(); it_packet != m_buffer_vector.end(); ++it_packet)
  {
    const auto& packet = *it_packet;
    sum += packet.getLength();
  }
  return static_cast<uint32_t>(sum);
}

}  // namespace data_processing
}  // namespace sick
