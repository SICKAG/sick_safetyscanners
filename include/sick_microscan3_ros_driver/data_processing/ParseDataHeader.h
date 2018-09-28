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
 * \file ParseDataHeader.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>

#include <sick_microscan3_ros_driver/datastructure/DataHeader.h>


namespace sick {
namespace data_processing {

class ParseDataHeader : public AbstractParseUDPSequence
{
public:
  ParseDataHeader();

  datastructure::DataHeader parseUDPSequence(sick::datastructure::PacketBuffer buffer,
                                             datastructure::Data& data);

private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  void setVersionIndicatorInDataHeader(const BYTE* data_ptr,
                                       datastructure::DataHeader& data_header);
  void setMajorVersionInDataHeader(const BYTE* data_ptr, datastructure::DataHeader& data_header);
  void setMinorVersionInDataHeader(const BYTE* data_ptr, datastructure::DataHeader& data_header);
  void setVersionReleaseInDataHeader(const BYTE* data_ptr, datastructure::DataHeader& data_header);
  void setSerialNumberOfDeviceInDataHeader(const BYTE* data_ptr,
                                           datastructure::DataHeader& data_header);
  void setSerialNumberOfSystemPlugInDataHeader(const BYTE* data_ptr,
                                               datastructure::DataHeader& data_header);
  void setChannelNumberInDataHeader(const BYTE* data_ptr, datastructure::DataHeader& data_header);
  void setSequenceNumberInDataHeader(const BYTE* data_ptr, datastructure::DataHeader& data_header);
  void setScanNumberInDataHeader(const BYTE* data_ptr, datastructure::DataHeader& data_header);
  void setTimestampDateInDataHeader(const BYTE* data_ptr, datastructure::DataHeader& data_header);
  void setTimestampTimeInDataHeader(const BYTE* data_ptr, datastructure::DataHeader& data_header);
  void setGeneralSystemStateBlockOffsetInDataHeader(const BYTE* data_ptr,
                                                    datastructure::DataHeader& data_header);
  void setGeneralSystemStateBlockSizeInDataHeader(const BYTE* data_ptr,
                                                  datastructure::DataHeader& data_header);
  void setDerivedValuesBlockOffsetInDataHeader(const BYTE* data_ptr,
                                               datastructure::DataHeader& data_header);
  void setDerivedValuesBlockSizeInDataHeader(const BYTE* data_ptr,
                                             datastructure::DataHeader& data_header);
  void setMeasurementDataBlockOffsetInDataHeader(const BYTE* data_ptr,
                                                 datastructure::DataHeader& data_header);
  void setMeasurementDataBlockSizeInDataHeader(const BYTE* data_ptr,
                                               datastructure::DataHeader& data_header);
  void setIntrusionDataBlockOffsetInDataHeader(const BYTE* data_ptr,
                                               datastructure::DataHeader& data_header);
  void setIntrusionDataBlockSizeInDataHeader(const BYTE* data_ptr,
                                             datastructure::DataHeader& data_header);
  void setApplicationDataBlockOffsetInDataHeader(const BYTE* data_ptr,
                                                 datastructure::DataHeader& data_header);
  void setApplicationDataBlockSizeInDataHeader(const BYTE* data_ptr,
                                               datastructure::DataHeader& data_header);
  void setDataInDataHeader(const BYTE* data_ptr, datastructure::DataHeader& data_header);
  void setVersionInDataHeader(const BYTE* data_ptr, datastructure::DataHeader& data_header);
  void setScanHeaderInDataHeader(const BYTE* data_ptr, datastructure::DataHeader& data_header);
  void setDataBlocksInDataHeader(const BYTE* data_ptr, datastructure::DataHeader& data_header);
};

} // namespace data_processing
} // namespace sick
