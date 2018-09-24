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
* \file ParseTCPPacket.h
*
* \author  Lennart Puck <puck@fzi.de>
* \date    2018-09-24
*/
//----------------------------------------------------------------------

#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>



namespace sick {

namespace cola2 {
class Command;
}

namespace data_processing {



class ParseTCPPacket : public AbstractParseUDPSequence
{
public:
  ParseTCPPacket();

  datastructure::IntrusionData parseUDPSequence(sick::datastructure::PacketBuffer buffer, datastructure::Data &data);

  bool parseTCPSequence(datastructure::PacketBuffer buffer, sick::cola2::Command &command);
  int getExpectedPacketLength(datastructure::PacketBuffer buffer);

  UINT16 getRequestID(datastructure::PacketBuffer buffer);

private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  UINT32 readSTx(datastructure::PacketBuffer &buffer);
  UINT32 readLength(datastructure::PacketBuffer &buffer);
  UINT16 readRequestID(datastructure::PacketBuffer &buffer);
  UINT8 readHubCntr(datastructure::PacketBuffer &buffer);
  UINT8 readNoC(datastructure::PacketBuffer &buffer);
  UINT32 readSessionID(datastructure::PacketBuffer &buffer);
  UINT8 readCommandType(datastructure::PacketBuffer &buffer);
  UINT8 readCommandMode(datastructure::PacketBuffer &buffer);
  UINT16 readErrorCode(datastructure::PacketBuffer &buffer);
  void readData(datastructure::PacketBuffer &buffer, std::vector<BYTE> &byteVector);
  bool setCommandValuesFromPacket(sick::datastructure::PacketBuffer &buffer,  sick::cola2::Command &command);
};

}
}


