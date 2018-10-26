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

#ifndef SICK_MICROSCAN3_ROS_DRIVER_DATASTRUCTURE_DATAGRAMHEADER_H
#define SICK_MICROSCAN3_ROS_DRIVER_DATASTRUCTURE_DATAGRAMHEADER_H

#include <stdint.h>

namespace sick {
namespace datastructure {


class DatagramHeader
{
public:
  static const uint32_t HEADER_SIZE = 24;

  DatagramHeader();

  uint32_t getDatagramMarker() const;
  void setDatagramMarker(const uint32_t& value);

  uint16_t getProtocol() const;
  void setProtocol(const uint16_t& value);

  uint8_t getMajorVersion() const;
  void setMajorVersion(const uint8_t& value);

  uint8_t getMinorVersion() const;
  void setMinorVersion(const uint8_t& value);

  uint32_t getTotalLength() const;
  void setTotalLength(const uint32_t& value);

  uint32_t getIdentification() const;
  void setIdentification(const uint32_t& value);

  uint32_t getFragmentOffset() const;
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

}  // namespace datastructure
}  // namespace sick

#endif
