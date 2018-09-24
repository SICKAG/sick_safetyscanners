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

TCPPaketMerger::TCPPaketMerger()
  : m_is_complete(false)
  , m_deployed_paket_buffer()
{

}


bool TCPPaketMerger::isComplete()
{
  return m_is_complete;
}

bool TCPPaketMerger::isEmpty()
{
  return m_buffer_vector.empty();
}

sick::datastructure::PacketBuffer TCPPaketMerger::getDeployedPacketBuffer()
{
  m_is_complete = false;
  return m_deployed_paket_buffer;
}

bool TCPPaketMerger::addTCPPacket(sick::datastructure::PacketBuffer buffer)
{
  if(isComplete()) {
      m_is_complete = false;
  }

  addToMap(buffer);
  deployPacketIfComplete();
  return isComplete();
}

bool TCPPaketMerger::addToMap(sick::datastructure::PacketBuffer newPacket)
{
  UINT32 currentSize = getCurrentSize();
  UINT32 remainingSize = m_targetSize - currentSize;
  m_buffer_vector.push_back(newPacket);
  if (remainingSize == newPacket.getLength())
  {
    m_is_complete = true;
  }

  return isComplete();

}

bool TCPPaketMerger::deployPacketIfComplete()
{
  if(isComplete()) {
    deployPacket();
    return true;
  }
  return false;
}

bool TCPPaketMerger::deployPacket()
{
  sick::datastructure::PacketBuffer::VectorBuffer headerless_packet_buffer;
  for (auto &parsed_packet_buffer : m_buffer_vector) {

    sick::datastructure::PacketBuffer packet_buffer = parsed_packet_buffer.getBuffer();

    headerless_packet_buffer.insert(headerless_packet_buffer.end(),
                                    packet_buffer.getBuffer().begin(),
                                    packet_buffer.getBuffer().end());
  }
  m_deployed_paket_buffer.setBuffer(headerless_packet_buffer);
  m_buffer_vector.clear();

}

UINT32 TCPPaketMerger::getTargetSize() const
{
  return m_targetSize;
}

void TCPPaketMerger::setTargetSize(const UINT32 &targetSize)
{
  m_targetSize = targetSize;
}

UINT32 TCPPaketMerger::getCurrentSize() const
{
  size_t sum = 0;
  for (auto it_packet = m_buffer_vector.begin();
     it_packet != m_buffer_vector.end(); ++it_packet)
  {
    const auto& packet = *it_packet;
    sum += packet.getLength();
  }
  return static_cast<UINT32>(sum);
}

}
}
