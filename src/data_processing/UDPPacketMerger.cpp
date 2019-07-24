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
 * \file UDPPacketMerger.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/UDPPacketMerger.h>

namespace sick {
namespace data_processing {

UDPPacketMerger::UDPPacketMerger()
  : m_is_complete(false)
{
}


bool UDPPacketMerger::isComplete() const
{
  return m_is_complete;
}

sick::datastructure::PacketBuffer UDPPacketMerger::getDeployedPacketBuffer()
{
  m_is_complete = false;
  return m_deployed_packet_buffer;
}

bool UDPPacketMerger::addUDPPacket(const datastructure::PacketBuffer& buffer)
{
  // Protect the internal memory for duplciate calls
  std::lock_guard<std::mutex> lock(m_buffer_mutex);

  if (isComplete())
  {
    m_is_complete = false;
  }
  sick::datastructure::DatagramHeader datagram_header;
  sick::data_processing::ParseDatagramHeader datagram_header_parser;
  datagram_header_parser.parseUDPSequence(buffer, datagram_header);
  addToMap(buffer, datagram_header);
  deployPacketIfComplete(datagram_header);

  return isComplete();
}

bool UDPPacketMerger::addToMap(const datastructure::PacketBuffer& buffer,
                               const datastructure::DatagramHeader& header)
{
  sick::datastructure::ParsedPacketBuffer parsed_packet_buffer(buffer, header);
  auto it = m_parsed_packet_buffer_map.find(header.getIdentification());
  if (it != m_parsed_packet_buffer_map.end())
  {
    it->second.push_back(parsed_packet_buffer);
  }
  else
  {
    sick::datastructure::ParsedPacketBuffer::ParsedPacketBufferVector vec;
    vec.push_back(parsed_packet_buffer);
    m_parsed_packet_buffer_map[header.getIdentification()] = vec;
  }
  return true;
}

bool UDPPacketMerger::deployPacketIfComplete(datastructure::DatagramHeader& header)
{
  auto it = m_parsed_packet_buffer_map.find(header.getIdentification());

  if (it == m_parsed_packet_buffer_map.end())
  {
    return false;
  }
  if (!checkIfComplete(header))
  {
    return false;
  }

  sick::datastructure::ParsedPacketBuffer::ParsedPacketBufferVector vec =
    getSortedParsedPacketBufferForIdentification(header);
  std::vector<uint8_t> headerless_packet_buffer = removeHeaderFromParsedPacketBuffer(vec);
  m_deployed_packet_buffer.setBuffer(headerless_packet_buffer);
  m_parsed_packet_buffer_map.erase(header.getIdentification());
  return true;
}

bool UDPPacketMerger::checkIfComplete(sick::datastructure::DatagramHeader& header)
{
  uint32_t total_length = header.getTotalLength();
  sick::datastructure::ParsedPacketBuffer::ParsedPacketBufferVector vec =
    getSortedParsedPacketBufferForIdentification(header);
  uint32_t cur_length = calcualteCurrentLengthOfParsedPacketBuffer(vec);
  if (cur_length != total_length)
  {
    return false;
  }
  m_is_complete = true;
  return true;
}

uint32_t UDPPacketMerger::calcualteCurrentLengthOfParsedPacketBuffer(
  const sick::datastructure::ParsedPacketBuffer::ParsedPacketBufferVector& vec)
{
  uint32_t cur_length = 0;

  for (auto& parsed_packet_buffer : vec)
  {
    sick::datastructure::PacketBuffer packet_buffer = parsed_packet_buffer.getPacketBuffer();
    cur_length += (packet_buffer.getLength() - sick::datastructure::DatagramHeader::HEADER_SIZE);
  }
  return cur_length;
}

sick::datastructure::ParsedPacketBuffer::ParsedPacketBufferVector
UDPPacketMerger::getSortedParsedPacketBufferForIdentification(
  const sick::datastructure::DatagramHeader& header)
{
  auto it = m_parsed_packet_buffer_map.find(header.getIdentification());
  sick::datastructure::ParsedPacketBuffer::ParsedPacketBufferVector vec = it->second;
  std::sort(
    vec.begin(), vec.end(), sick::datastructure::ParsedPacketBuffer::sortForIncreasingOffset);
  return vec;
}

std::vector<uint8_t> UDPPacketMerger::removeHeaderFromParsedPacketBuffer(
  const sick::datastructure::ParsedPacketBuffer::ParsedPacketBufferVector& vec)
{
  std::vector<uint8_t> headerless_packet_buffer;
  for (auto& parsed_packet_buffer : vec)
  {
    sick::datastructure::PacketBuffer packet_buffer = parsed_packet_buffer.getPacketBuffer();

    // This insert is memory safe because we constructed the buffer in this function
    const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = packet_buffer.getBuffer();
    headerless_packet_buffer.insert(headerless_packet_buffer.end(),
                                    vec_ptr->begin() +
                                      sick::datastructure::DatagramHeader::HEADER_SIZE,
                                    vec_ptr->end());
  }
  return headerless_packet_buffer;
}


} // namespace data_processing
} // namespace sick
