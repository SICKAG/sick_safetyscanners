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
 * \file ParsedPacketBuffer.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-26
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_PARSEDPACKETBUFFER_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_PARSEDPACKETBUFFER_H

#include <iostream>
#include <stdint.h>
#include <string>
#include <vector>

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <sick_safetyscanners/datastructure/DatagramHeader.h>
#include <sick_safetyscanners/datastructure/PacketBuffer.h>


namespace sick {
namespace datastructure {


/*!
 * \brief Struct of a PacketBuffer with a parsed header
 */
class ParsedPacketBuffer
{
public:
  typedef std::vector<ParsedPacketBuffer> ParsedPacketBufferVector;

  /*!
   * \brief Constructor of ParsedPacketBuffer.
   * \param packet_buffer: Input Packetbuffer
   * \param datagram_header: Input parsed header of PacketBuffer.
   */
  ParsedPacketBuffer(const sick::datastructure::PacketBuffer& packet_buffer,
                     sick::datastructure::DatagramHeader datagram_header);

  /*!
   * \brief Static function to sort ParsedPacketBuffers.
   * \param ppb1 First ParsedPacketBuffer.
   * \param ppb2 Second ParsedPacketBuffer.
   * \return If first is smaller then second.
   */
  static bool sortForIncreasingOffset(const ParsedPacketBuffer& ppb1,
                                      const ParsedPacketBuffer& ppb2)
  {
    return ppb1.m_datagram_header.getFragmentOffset() < ppb2.m_datagram_header.getFragmentOffset();
  }

  /*!
   * \brief Returns the contains packet buffer of the parsed packet buffer.
   * \return The packet buffer, contained in the parsed packet buffer.
   */
  sick::datastructure::PacketBuffer getPacketBuffer() const;

  /*!
   * \brief Sets the packet buffer in the parsed packet buffer.
   * \param packet_buffer The new packet buffer for the parsed packet buffer.
   */
  void setPacketBuffer(const sick::datastructure::PacketBuffer& packet_buffer);

private:
  /*!
   * \brief PacketBuffer
   */
  sick::datastructure::PacketBuffer m_packet_buffer;

  /*!
   * \brief Parsed datagramheader
   */
  sick::datastructure::DatagramHeader m_datagram_header;
};


} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_PACKETBUFFER_H
