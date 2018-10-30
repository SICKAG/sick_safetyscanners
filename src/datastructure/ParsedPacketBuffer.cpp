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
 * \file ParsedPacketBuffer.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-26
 */
//----------------------------------------------------------------------

#include "sick_safetyscanners/datastructure/ParsedPacketBuffer.h"

namespace sick {
namespace datastructure {

ParsedPacketBuffer::ParsedPacketBuffer(const sick::datastructure::PacketBuffer& packet_buffer,
                                       DatagramHeader datagram_header)
  : m_packet_buffer(packet_buffer)
  , m_datagram_header(datagram_header)
{
}

sick::datastructure::PacketBuffer ParsedPacketBuffer::getPacketBuffer() const
{
  return m_packet_buffer;
}

void ParsedPacketBuffer::setPacketBuffer(const sick::datastructure::PacketBuffer& packet_buffer)
{
  m_packet_buffer = packet_buffer;
}


} // namespace datastructure
} // namespace sick
