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

/*!
 * \brief MAXSIZE of the packetbuffer
 */
const int MAXSIZE = 10000;

/*!
 * \brief A packetbuffer for the raw data from the sensor.
 */
class PacketBuffer
{
public:

  /*!
   * \brief Typedef for an arraybuffer which can be read from the sensor.
   */
  typedef boost::array<uint8_t, MAXSIZE> ArrayBuffer;
  /*!
   * \brief Typedef for a vector buffer, to sort the incoming packets.
   */
  typedef std::vector<uint8_t> VectorBuffer;

  /*!
   * \brief Constructor of PacketBuffer.
   */
  PacketBuffer();

  /*!
   * \brief Constructor of PacketBuffer.
   * \param buffer Vectorbuffer to set for the PacketBuffer.
   */
  PacketBuffer(const VectorBuffer& buffer);

  /*!
   * \brief Constructor of PacketBuffer.
   * \param buffer Arraybuffer to set for the PacketBuffer.
   * \param length Length of the array.
   */
  PacketBuffer(const ArrayBuffer& buffer, const size_t &length);

  /*!
   * \brief Returns defined maximum size of PacketBuffer.
   * \return Maximum size of PacketBuffer.
   */
  static uint32_t getMaxSize() { return MAXSIZE; }

  /*!
   * \brief Getter to return the VectorBuffer saved in the PacketBuffer.
   * \return VectorBuffer
   */
  const VectorBuffer& getBuffer() const;

  /*!
   * \brief Setter for the PacketBuffer.
   * \param buffer Input VectorBuffer to save.
   */
  void setBuffer(const VectorBuffer& buffer);

  /*!
   * \brief Setter for the PacketBuffer.
   * \param buffer Input ArrayBuffer to save.
   * \param length Length of input ArrayBuffer.
   */
  void setBuffer(const ArrayBuffer& buffer, const size_t &length);

  /*!
   * \brief Returns length of the current PacketBuffer.
   * \return Length.
   */
  size_t getLength() const;


private:
  VectorBuffer m_buffer;
};

/*!
 * \brief Struct of a PacketBuffer with a parsed header
 */
struct ParsedPacketBuffer
{
  /*!
   * \brief Constructor of struct for ParsedPacketBuffer.
   * \param packet_buffer: Input Packetbuffer
   * \param datagram_header: Input parsed header of PacketBuffer.
   */
  ParsedPacketBuffer(const sick::datastructure::PacketBuffer& packet_buffer,
                     sick::datastructure::DatagramHeader datagram_header)
    : m_packet_buffer(packet_buffer)
    , m_datagram_header(datagram_header)
  {
  }
  /*!
   * \brief PacketBuffer
   */
  sick::datastructure::PacketBuffer m_packet_buffer;

  /*!
   * \brief Parsed datagramheader
   */
  sick::datastructure::DatagramHeader m_datagram_header;
};
typedef std::vector<ParsedPacketBuffer> ParsedPacketBufferVector;

/*!
 * \brief Static function to sort ParsedPacketBuffers.
 * \param ppb1 First ParsedPacketBuffer.
 * \param ppb2 Second ParsedPacketBuffer.
 * \return If first is smaller then second.
 */
static bool sortForIncreasingOffset(const ParsedPacketBuffer& ppb1, const ParsedPacketBuffer& ppb2)
{
  return ppb1.m_datagram_header.getFragmentOffset() < ppb2.m_datagram_header.getFragmentOffset();
}

} // namespace datastructure
} // namespace sick

#endif
