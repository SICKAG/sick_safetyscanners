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

#pragma once

#include <sick_microscan3_ros_driver/datastructure/DatagramHeader.h>

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>

namespace sick {
namespace data_processing {

class ParseDatagramHeader : public AbstractParseUDPSequence
{
public:
  ParseDatagramHeader();
  bool parseUDPSequence(sick::datastructure::PacketBuffer buffer,
                        sick::datastructure::DatagramHeader& header);

private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;

  bool setDataInHeader(const BYTE* data_ptr, datastructure::DatagramHeader& header);

  bool setDatagramMarkerInHeader(const BYTE* data_ptr, datastructure::DatagramHeader& header);
  bool setProtocolInHeader(const BYTE* data_ptr, datastructure::DatagramHeader& header);
  bool setMajorVersionInHeader(const BYTE* data_ptr, datastructure::DatagramHeader& header);
  bool setMinorVersionInHeader(const BYTE* data_ptr, datastructure::DatagramHeader& header);
  bool setTotalLengthInHeader(const BYTE* data_ptr, datastructure::DatagramHeader& header);
  bool setIdentificationInHeader(const BYTE* data_ptr, datastructure::DatagramHeader& header);
  bool setFragmentOffsetInHeader(const BYTE* data_ptr, datastructure::DatagramHeader& header);
};

} // namespace data_processing
} // namespace sick
