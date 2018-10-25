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
 * \file ParseDataHeader.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_microscan3_ros_driver/data_processing/ParseDataHeader.h>

namespace sick {
namespace data_processing {

ParseDataHeader::ParseDataHeader()
{
  m_reader_ptr = std::make_shared<sick::data_processing::ReadWriteHelper>();
}

datastructure::DataHeader ParseDataHeader::parseUDPSequence(const datastructure::PacketBuffer &buffer,
                                                            datastructure::Data& data) const
{
  const uint8_t* data_ptr(buffer.getBuffer().data());
  datastructure::DataHeader data_header;
  setDataInDataHeader(data_ptr, data_header);
  return data_header;
}

void ParseDataHeader::setDataInDataHeader(const uint8_t* &data_ptr,
                                          datastructure::DataHeader& data_header) const
{
  setVersionInDataHeader(data_ptr, data_header);
  setScanHeaderInDataHeader(data_ptr, data_header);
  setDataBlocksInDataHeader(data_ptr, data_header);
}


void ParseDataHeader::setVersionInDataHeader(const uint8_t* &data_ptr,
                                             datastructure::DataHeader& data_header) const
{
  setVersionIndicatorInDataHeader(data_ptr, data_header);
  setMajorVersionInDataHeader(data_ptr, data_header);
  setMinorVersionInDataHeader(data_ptr, data_header);
  setVersionReleaseInDataHeader(data_ptr, data_header);
  setSerialNumberOfDeviceInDataHeader(data_ptr, data_header);
  setSerialNumberOfSystemPlugInDataHeader(data_ptr, data_header);
}

void ParseDataHeader::setScanHeaderInDataHeader(const uint8_t* &data_ptr,
                                                datastructure::DataHeader& data_header) const
{
  setChannelNumberInDataHeader(data_ptr, data_header);
  setSequenceNumberInDataHeader(data_ptr, data_header);
  setScanNumberInDataHeader(data_ptr, data_header);
  setTimestampDateInDataHeader(data_ptr, data_header);
  setTimestampTimeInDataHeader(data_ptr, data_header);
}

void ParseDataHeader::setDataBlocksInDataHeader(const uint8_t* &data_ptr,
                                                datastructure::DataHeader& data_header) const
{
  setGeneralSystemStateBlockOffsetInDataHeader(data_ptr, data_header);
  setGeneralSystemStateBlockSizeInDataHeader(data_ptr, data_header);
  setDerivedValuesBlockOffsetInDataHeader(data_ptr, data_header);
  setDerivedValuesBlockSizeInDataHeader(data_ptr, data_header);
  setMeasurementDataBlockOffsetInDataHeader(data_ptr, data_header);
  setMeasurementDataBlockSizeInDataHeader(data_ptr, data_header);
  setIntrusionDataBlockOffsetInDataHeader(data_ptr, data_header);
  setIntrusionDataBlockSizeInDataHeader(data_ptr, data_header);
  setApplicationDataBlockOffsetInDataHeader(data_ptr, data_header);
  setApplicationDataBlockSizeInDataHeader(data_ptr, data_header);
}

void ParseDataHeader::setVersionIndicatorInDataHeader(const uint8_t* &data_ptr,
                                                      datastructure::DataHeader& data_header) const
{
  data_header.setVersionIndicator(m_reader_ptr->readuint8_tLittleEndian(data_ptr, 0));
}

void ParseDataHeader::setMajorVersionInDataHeader(const uint8_t* &data_ptr,
                                                  datastructure::DataHeader& data_header) const
{
  data_header.setVersionMajorVersion(m_reader_ptr->readuint8_tLittleEndian(data_ptr, 1));
}

void ParseDataHeader::setMinorVersionInDataHeader(const uint8_t* &data_ptr,
                                                  datastructure::DataHeader& data_header) const
{
  data_header.setVersionMinorVersion(m_reader_ptr->readuint8_tLittleEndian(data_ptr, 2));
}

void ParseDataHeader::setVersionReleaseInDataHeader(const uint8_t* &data_ptr,
                                                    datastructure::DataHeader& data_header) const
{
  data_header.setVersionRelease(m_reader_ptr->readuint8_tLittleEndian(data_ptr, 3));
}

void ParseDataHeader::setSerialNumberOfDeviceInDataHeader(const uint8_t* &data_ptr,
                                                          datastructure::DataHeader& data_header) const
{
  data_header.setSerialNumberOfDevice(m_reader_ptr->readuint32_tLittleEndian(data_ptr, 4));
}

void ParseDataHeader::setSerialNumberOfSystemPlugInDataHeader(
  const uint8_t* &data_ptr, datastructure::DataHeader& data_header) const
{
  data_header.setSerialNumberOfSystemPlug(m_reader_ptr->readuint32_tLittleEndian(data_ptr, 8));
}

void ParseDataHeader::setChannelNumberInDataHeader(const uint8_t* &data_ptr,
                                                   datastructure::DataHeader& data_header) const
{
  data_header.setChannelNumber(m_reader_ptr->readuint8_tLittleEndian(data_ptr, 12));
}

void ParseDataHeader::setSequenceNumberInDataHeader(const uint8_t* &data_ptr,
                                                    datastructure::DataHeader& data_header) const
{
  data_header.setSequenceNumber(m_reader_ptr->readuint32_tLittleEndian(data_ptr, 16));
}

void ParseDataHeader::setScanNumberInDataHeader(const uint8_t* &data_ptr,
                                                datastructure::DataHeader& data_header) const
{
  data_header.setScanNumber(m_reader_ptr->readuint32_tLittleEndian(data_ptr, 20));
}

void ParseDataHeader::setTimestampDateInDataHeader(const uint8_t* &data_ptr,
                                                   datastructure::DataHeader& data_header) const
{
  data_header.setTimestampDate(m_reader_ptr->readuint16_tLittleEndian(data_ptr, 24));
}

void ParseDataHeader::setTimestampTimeInDataHeader(const uint8_t* &data_ptr,
                                                   datastructure::DataHeader& data_header) const
{
  data_header.setTimestampTime(m_reader_ptr->readuint32_tLittleEndian(data_ptr, 28));
}

void ParseDataHeader::setGeneralSystemStateBlockOffsetInDataHeader(
  const uint8_t* &data_ptr, datastructure::DataHeader& data_header) const
{
  data_header.setGeneralSystemStateBlockOffset(m_reader_ptr->readuint16_tLittleEndian(data_ptr, 32));
}

void ParseDataHeader::setGeneralSystemStateBlockSizeInDataHeader(
  const uint8_t* &data_ptr, datastructure::DataHeader& data_header) const
{
  data_header.setGeneralSystemStateBlockSize(m_reader_ptr->readuint16_tLittleEndian(data_ptr, 34));
}

void ParseDataHeader::setDerivedValuesBlockOffsetInDataHeader(
  const uint8_t* &data_ptr, datastructure::DataHeader& data_header) const
{
  data_header.setDerivedValuesBlockOffset(m_reader_ptr->readuint16_tLittleEndian(data_ptr, 36));
}

void ParseDataHeader::setDerivedValuesBlockSizeInDataHeader(const uint8_t* &data_ptr,
                                                            datastructure::DataHeader& data_header) const
{
  data_header.setDerivedValuesBlockSize(m_reader_ptr->readuint16_tLittleEndian(data_ptr, 38));
}

void ParseDataHeader::setMeasurementDataBlockOffsetInDataHeader(
  const uint8_t* &data_ptr, datastructure::DataHeader& data_header) const
{
  data_header.setMeasurementDataBlockOffset(m_reader_ptr->readuint16_tLittleEndian(data_ptr, 40));
}

void ParseDataHeader::setMeasurementDataBlockSizeInDataHeader(
  const uint8_t* &data_ptr, datastructure::DataHeader& data_header) const
{
  data_header.setMeasurementDataBlockSize(m_reader_ptr->readuint16_tLittleEndian(data_ptr, 42));
}

void ParseDataHeader::setIntrusionDataBlockOffsetInDataHeader(
  const uint8_t* &data_ptr, datastructure::DataHeader& data_header) const
{
  data_header.setIntrusionDataBlockOffset(m_reader_ptr->readuint16_tLittleEndian(data_ptr, 44));
}

void ParseDataHeader::setIntrusionDataBlockSizeInDataHeader(const uint8_t* &data_ptr,
                                                            datastructure::DataHeader& data_header) const
{
  data_header.setIntrusionDataBlockSize(m_reader_ptr->readuint16_tLittleEndian(data_ptr, 46));
}

void ParseDataHeader::setApplicationDataBlockOffsetInDataHeader(
  const uint8_t* &data_ptr, datastructure::DataHeader& data_header) const
{
  data_header.setApplicationDataBlockOffset(m_reader_ptr->readuint16_tLittleEndian(data_ptr, 48));
}

void ParseDataHeader::setApplicationDataBlockSizeInDataHeader(
  const uint8_t* &data_ptr, datastructure::DataHeader& data_header) const
{
  data_header.setApplicationDataBlockSize(m_reader_ptr->readuint16_tLittleEndian(data_ptr, 50));
}

} // namespace data_processing
} // namespace sick
