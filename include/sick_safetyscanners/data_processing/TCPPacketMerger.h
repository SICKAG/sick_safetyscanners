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
 * \file TCPPacketMerger.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATA_PROCESSING_TCPPACKETMERGER_H
#define SICK_SAFETYSCANNERS_DATA_PROCESSING_TCPPACKETMERGER_H

#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/data_processing/ParseDatagramHeader.h>

#include <mutex>
#include <vector>

namespace sick {
namespace data_processing {

/*!
 * \brief Merges incoming tcp packets together to get a complete data packet.
 */
class TCPPacketMerger
{
public:
  /*!
   * \brief Constructor of merger.
   */
  TCPPacketMerger();

  /*!
   * \brief Check if the packet is complete.
   *
   * \returns True if the packet is complete.
   */
  bool isComplete() const;

  /*!
   * \brief Checks if the buffer vector is empty.
   *
   * \returns True if the buffer vector is empty.
   */
  bool isEmpty() const;

  /*!
   * \brief Adds a new tcp packet to the merger. Returns true if this tcp packet completes a data
   * packet.
   *
   * \param buffer The new tcp packet.
   *
   * \returns True if the data packet is complete with the new packet.
   */
  bool addTCPPacket(const sick::datastructure::PacketBuffer& buffer);

  /*!
   * \brief Gets the latest complete data packet.
   *
   * \returns The latest complete data packet.
   */
  sick::datastructure::PacketBuffer getDeployedPacketBuffer();

  /*!
   * \brief Returns the expected target size of a complete data packet.
   *
   * \returns The target size of a complete data packet.
   */
  uint32_t getTargetSize() const;

  /*!
   * \brief Sets the target size of a data packet.
   *
   * \param targetSize The new target size of a data packet.
   */
  void setTargetSize(const uint32_t& targetSize);

private:
  bool m_is_complete;
  sick::datastructure::PacketBuffer m_deployed_packet_buffer;

  std::vector<sick::datastructure::PacketBuffer> m_buffer_vector;
  std::mutex m_buffer_mutex;
  uint32_t m_targetSize;

  bool addToMap(const sick::datastructure::PacketBuffer& new_packet);
  bool deployPacketIfComplete();

  uint32_t getCurrentSize();
  bool deployPacket();
};

} // namespace data_processing
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATA_PROCESSING_TCPPACKETMERGER_H
