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
 * \file DatagramHeader.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/datastructure/DatagramHeader.h>

namespace sick {
namespace datastructure {

DatagramHeader::DatagramHeader() {}

uint32_t DatagramHeader::getDatagramMarker() const
{
  return m_datagram_marker;
}

void DatagramHeader::setDatagramMarker(const uint32_t& value)
{
  m_datagram_marker = value;
}

uint16_t DatagramHeader::getProtocol() const
{
  return m_protocol;
}

void DatagramHeader::setProtocol(const uint16_t& value)
{
  m_protocol = value;
}

uint8_t DatagramHeader::getMajorVersion() const
{
  return m_major_version;
}

void DatagramHeader::setMajorVersion(const uint8_t& value)
{
  m_major_version = value;
}

uint8_t DatagramHeader::getMinorVersion() const
{
  return m_minor_version;
}

void DatagramHeader::setMinorVersion(const uint8_t& value)
{
  m_minor_version = value;
}

uint32_t DatagramHeader::getTotalLength() const
{
  return m_total_length;
}

void DatagramHeader::setTotalLength(const uint32_t& value)
{
  m_total_length = value;
}

uint32_t DatagramHeader::getIdentification() const
{
  return m_identification;
}

void DatagramHeader::setIdentification(const uint32_t& value)
{
  m_identification = value;
}

uint32_t DatagramHeader::getFragmentOffset() const
{
  return m_fragment_offset;
}

void DatagramHeader::setFragmentOffset(const uint32_t& value)
{
  m_fragment_offset = value;
}

} // namespace datastructure
} // namespace sick
