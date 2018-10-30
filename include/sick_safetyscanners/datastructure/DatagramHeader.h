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
 * \file DatagramHeader.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_DATASTRUCTURE_DATAGRAMHEADER_H
#define SICK_SAFETYSCANNERS_DATASTRUCTURE_DATAGRAMHEADER_H

#include <stdint.h>

namespace sick {
namespace datastructure {


/*!
 * \brief Contains the contents of a udp datagram header. Used to match the datagrams together to
 * form a complete data packet.
 */
class DatagramHeader
{
public:
  static const uint32_t HEADER_SIZE = 24;

  /*!
   * \brief Constructor of the datagram header.
   */
  DatagramHeader();

  /*!
   * \brief Gets the datagram marker.
   *
   * \returns  The datagram marker.
   */
  uint32_t getDatagramMarker() const;
  /*!
   * \brief Sets the datagram marker.
   *
   * \param value The new datagram marker.
   */
  void setDatagramMarker(const uint32_t& value);

  /*!
   * \brief Gets the used Protocol.
   *
   * \returns The protocol.
   */
  uint16_t getProtocol() const;
  /*!
   * \brief Sets the used protocol.
   *
   * \param value The used protocol.
   */
  void setProtocol(const uint16_t& value);

  /*!
   * \brief Gets the major version number.
   *
   * \returns  The major version number.
   */
  uint8_t getMajorVersion() const;
  /*!
   * \brief Sets the major version number.
   *
   * \param value The major version number.
   */
  void setMajorVersion(const uint8_t& value);

  /*!
   * \brief Gets the minor version number.
   *
   * \returns The minor version number.
   */
  uint8_t getMinorVersion() const;
  /*!
   * \brief Sets the minor version number.
   *
   * \param value The minor version number.
   */
  void setMinorVersion(const uint8_t& value);

  /*!
   * \brief Gets the total length of the data packet.
   *
   * Total length of the (possibly fragmented) measurement data instance (excluding headers).
   *
   * \returns The total length.
   */
  uint32_t getTotalLength() const;
  /*!
   * \brief Sets the total length of the data packet.
   *
   * \param value The total length of the data packet.
   */
  void setTotalLength(const uint32_t& value);

  /*!
   * \brief Gets the identification of the data.
   *
   * Datagrams (fragments) that belong to the same
   * measurement data output instance share the same identifier. The number will increase
   * with each measurement data instance generated per channel.
   *
   * \returns The identification.
   */
  uint32_t getIdentification() const;
  /*!
   * \brief Sets the identification of the data.
   *
   * \param value The identification.
   */
  void setIdentification(const uint32_t& value);

  /*!
   * \brief Gets the fragment offset of the data.
   *
   * Offset (in bytes) of the measurement data carried in this datagram (fragment) relative to
   * the start of the overall measurement data output instance.
   *
   * \returns The fragment offset.
   */
  uint32_t getFragmentOffset() const;
  /*!
   * \brief Sets the fragment offset of the data.
   *
   * \param value The fragment offset.
   */
  void setFragmentOffset(const uint32_t& value);

private:
  uint32_t m_datagram_marker;
  uint16_t m_protocol;
  uint8_t m_major_version;
  uint8_t m_minor_version;
  uint32_t m_total_length;
  uint32_t m_identification;
  uint32_t m_fragment_offset;
};

} // namespace datastructure
} // namespace sick

#endif // SICK_SAFETYSCANNERS_DATASTRUCTURE_DATAGRAMHEADER_H
