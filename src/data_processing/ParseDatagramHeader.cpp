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
 * \file ParseDatagramHeader.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_microscan3_ros_driver/data_processing/ParseDatagramHeader.h>

namespace sick {
namespace data_processing {

ParseDatagramHeader::ParseDatagramHeader()
{
  m_reader_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();
}

bool ParseDatagramHeader::parseUDPSequence(datastructure::PacketBuffer buffer,
                                           datastructure::DatagramHeader& header)
{
  const BYTE* data_ptr(buffer.getBuffer().data());
  setDataInHeader(data_ptr, header);
  return true;
}

bool ParseDatagramHeader::setDataInHeader(const BYTE* data_ptr,
                                          datastructure::DatagramHeader& header)
{
  setDatagramMarkerInHeader(data_ptr, header);
  setProtocolInHeader(data_ptr, header);
  setMajorVersionInHeader(data_ptr, header);
  setMinorVersionInHeader(data_ptr, header);
  setTotalLengthInHeader(data_ptr, header);
  setIdentificationInHeader(data_ptr, header);
  setFragmentOffsetInHeader(data_ptr, header);
}

bool ParseDatagramHeader::setDatagramMarkerInHeader(const BYTE* data_ptr,
                                                    datastructure::DatagramHeader& header)
{
  header.setDatagramMarker(m_reader_ptr->readUINT32BigEndian(data_ptr, 0));
}

bool ParseDatagramHeader::setProtocolInHeader(const BYTE* data_ptr,
                                              datastructure::DatagramHeader& header)
{
  header.setProtocol(m_reader_ptr->readUINT16BigEndian(data_ptr, 4));
}

bool ParseDatagramHeader::setMajorVersionInHeader(const BYTE* data_ptr,
                                                  datastructure::DatagramHeader& header)
{
  header.setMajorVersion(m_reader_ptr->readUINT8LittleEndian(data_ptr, 6));
}

bool ParseDatagramHeader::setMinorVersionInHeader(const BYTE* data_ptr,
                                                  datastructure::DatagramHeader& header)
{
  header.setMinorVersion(m_reader_ptr->readUINT8LittleEndian(data_ptr, 7));
}

bool ParseDatagramHeader::setTotalLengthInHeader(const BYTE* data_ptr,
                                                 datastructure::DatagramHeader& header)
{
  header.setTotalLength(m_reader_ptr->readUINT32LittleEndian(data_ptr, 8));
}

bool ParseDatagramHeader::setIdentificationInHeader(const BYTE* data_ptr,
                                                    datastructure::DatagramHeader& header)
{
  header.setIdentification(m_reader_ptr->readUINT32LittleEndian(data_ptr, 12));
}

bool ParseDatagramHeader::setFragmentOffsetInHeader(const BYTE* data_ptr,
                                                    datastructure::DatagramHeader& header)
{
  header.setFragmentOffset(m_reader_ptr->readUINT32LittleEndian(data_ptr, 16));
}

} // namespace data_processing
} // namespace sick
