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
 * \file PacketBuffer.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef PACKETBUFFER_H
#define PACKETBUFFER_H

#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <sick_microscan3_ros_driver/datastructure/DatagramHeader.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>


namespace sick {
namespace datastructure {

const int MAXSIZE = 10000;


class PacketBuffer
{
public:
  typedef uint8_t array_type;
  typedef boost::array<array_type, MAXSIZE> ArrayBuffer;
  typedef std::vector<array_type> VectorBuffer;

  PacketBuffer();
  PacketBuffer(const VectorBuffer& buffer);
  PacketBuffer(const ArrayBuffer& buffer, size_t length);

  static uint32_t getMaxSize() { return MAXSIZE; }

  const VectorBuffer& getBuffer() const;
  void setBuffer(const VectorBuffer& buffer);
  void setBuffer(const ArrayBuffer& buffer, size_t length);

  size_t getLength() const;


private:
  VectorBuffer m_buffer;
};

struct ParsedPacketBuffer
{
  ParsedPacketBuffer(const sick::datastructure::PacketBuffer& packet_buffer,
                     sick::datastructure::DatagramHeader datagram_header)
    : m_packet_buffer(packet_buffer)
    , m_datagram_header(datagram_header)
  {
  }
  sick::datastructure::PacketBuffer m_packet_buffer;
  sick::datastructure::DatagramHeader m_datagram_header;
};
typedef std::vector<ParsedPacketBuffer> ParsedPacketBufferVector;

static bool sortForIncreasingOffset(const ParsedPacketBuffer& ppb1, const ParsedPacketBuffer& ppb2)
{
  return ppb1.m_datagram_header.getFragmentOffset() < ppb2.m_datagram_header.getFragmentOffset();
}

} // namespace datastructure
} // namespace sick

#endif
