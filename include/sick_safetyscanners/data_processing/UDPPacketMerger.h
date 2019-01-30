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
 * \file UDPPacketMerger.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_UDPPACKETMERGER_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_UDPPACKETMERGER_H

#include <sick_safetyscanners/datastructure/PacketBuffer.h>
#include <sick_safetyscanners/datastructure/ParsedPacketBuffer.h>

#include <sick_safetyscanners/data_processing/ParseDatagramHeader.h>

#include <algorithm>
#include <mutex>

namespace sick {
namespace data_processing {

/*!
 * \brief Merges udp packets together to a complete data packet.
 */
class UDPPacketMerger
{
public:
  /*!
   * \brief Constructor of udp packet merger.
   */
  UDPPacketMerger();

  /*!
   * \brief Returns if a data packet is complete.
   *
   * \returns If a data packet is complete.
   */
  bool isComplete() const;

  /*!
   * \brief Adds a udp packet to the packet map, when a data packet is complete it will return true.
   *
   * \param buffer The new udp packet.
   *
   * \returns True if a data packet is complete.
   */
  bool addUDPPacket(const sick::datastructure::PacketBuffer& buffer);

  /*!
   * \brief Get the latest complete data packet.
   *
   * \returns The latest complete data packet.
   */
  sick::datastructure::PacketBuffer getDeployedPacketBuffer();

private:
  bool m_is_complete;
  sick::datastructure::PacketBuffer m_deployed_packet_buffer;

  std::map<uint32_t, sick::datastructure::ParsedPacketBuffer::ParsedPacketBufferVector>
    m_parsed_packet_buffer_map;
  std::mutex m_buffer_mutex;

  bool addToMap(const sick::datastructure::PacketBuffer& buffer,
                const sick::datastructure::DatagramHeader& header);
  bool deployPacketIfComplete(datastructure::DatagramHeader& header);
  bool checkIfComplete(sick::datastructure::DatagramHeader& header);
  uint32_t calcualteCurrentLengthOfParsedPacketBuffer(
    const sick::datastructure::ParsedPacketBuffer::ParsedPacketBufferVector& vec);
  sick::datastructure::ParsedPacketBuffer::ParsedPacketBufferVector
  getSortedParsedPacketBufferForIdentification(const sick::datastructure::DatagramHeader& header);
  std::vector<uint8_t> removeHeaderFromParsedPacketBuffer(
    const sick::datastructure::ParsedPacketBuffer::ParsedPacketBufferVector& vec);
};

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_UDPPACKETMERGER_H
