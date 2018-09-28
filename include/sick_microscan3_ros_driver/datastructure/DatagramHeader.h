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

#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>

namespace sick {
namespace datastructure {


class DatagramHeader
{
public:
  static const UINT32 HEADER_SIZE = 24;

  DatagramHeader();

  UINT32 getDatagramMarker() const;
  void setDatagramMarker(const UINT32& value);

  UINT16 getProtocol() const;
  void setProtocol(const UINT16& value);

  UINT8 getMajorVersion() const;
  void setMajorVersion(const UINT8& value);

  UINT8 getMinorVersion() const;
  void setMinorVersion(const UINT8& value);

  UINT32 getTotalLength() const;
  void setTotalLength(const UINT32& value);

  UINT32 getIdentification() const;
  void setIdentification(const UINT32& value);

  UINT32 getFragmentOffset() const;
  void setFragmentOffset(const UINT32& value);

private:
  UINT32 m_datagram_marker;
  UINT16 m_protocol;
  UINT8 m_major_version;
  UINT8 m_minor_version;
  UINT32 m_total_length;
  UINT32 m_identification;
  UINT32 m_fragment_offset;
};

} // namespace datastructure
} // namespace sick
