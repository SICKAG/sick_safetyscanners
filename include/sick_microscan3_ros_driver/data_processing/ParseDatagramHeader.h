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
 * \file ParseDatagramHeader.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSEDATAGRAMHEADER_H
#define SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSEDATAGRAMHEADER_H

#include <sick_microscan3_ros_driver/datastructure/Data.h>
#include <sick_microscan3_ros_driver/datastructure/DatagramHeader.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>

#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>

namespace sick {
namespace data_processing {

class ParseDatagramHeader
{
public:
  ParseDatagramHeader();
  bool parseUDPSequence(const datastructure::PacketBuffer& buffer,
                        sick::datastructure::DatagramHeader& header) const;

private:
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;

  void setDataInHeader(const uint8_t*& data_ptr, datastructure::DatagramHeader& header) const;

  void setDatagramMarkerInHeader(const uint8_t*& data_ptr,
                                 datastructure::DatagramHeader& header) const;
  void setProtocolInHeader(const uint8_t*& data_ptr, datastructure::DatagramHeader& header) const;
  void setMajorVersionInHeader(const uint8_t*& data_ptr,
                               datastructure::DatagramHeader& header) const;
  void setMinorVersionInHeader(const uint8_t*& data_ptr,
                               datastructure::DatagramHeader& header) const;
  void setTotalLengthInHeader(const uint8_t*& data_ptr,
                              datastructure::DatagramHeader& header) const;
  void setIdentificationInHeader(const uint8_t*& data_ptr,
                                 datastructure::DatagramHeader& header) const;
  void setFragmentOffsetInHeader(const uint8_t*& data_ptr,
                                 datastructure::DatagramHeader& header) const;
};

}  // namespace data_processing
}  // namespace sick

#endif
