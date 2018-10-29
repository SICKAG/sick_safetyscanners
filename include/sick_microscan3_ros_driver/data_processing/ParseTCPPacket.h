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

#ifndef SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSETCPPACKET_H
#define SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSETCPPACKET_H

#include <sick_microscan3_ros_driver/datastructure/Data.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>
#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>

#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>

namespace sick {

namespace cola2 {
  /*!
   * \brief Forward declaration of Command class.
   */
class Command;
}

namespace data_processing {


  /*!
   * \brief Parser for an incoming TCP packet.
   */
class ParseTCPPacket
{
public:
  /*!
   * \brief Constructor of parser.
   */
  ParseTCPPacket();

  /*!
   * \brief Parse the tcp sequence to get the header information of the cola2 protocol.
   *
   * \param buffer The incoming tcp connection.
   * \param command Reference to the command and set the returned method type and mode and the data. 
   *
   * \returns If parsing was successful.
   */
  bool parseTCPSequence(const datastructure::PacketBuffer& buffer,
                        sick::cola2::Command& command) const;

  /*!
   * \brief Gets the expected packet length for a buffer.
   *
   * \param buffer The incoming tcp packet.
   *
   * \returns Expected length of the incoming packet buffer.
   */
  uint32_t getExpectedPacketLength(const datastructure::PacketBuffer& buffer);

  /*!
   * \brief Gets the request ID of the incoming tcp packet.
   *
   * \param buffer The incoming tcp packet.
   *
   * \returns The request ID of the incoming packet buffer.
   */
  uint16_t getRequestID(const datastructure::PacketBuffer& buffer) const;

private:
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  uint32_t readSTx(const uint8_t*& data_ptr) const;
  uint32_t readLength(const uint8_t*& data_ptr) const;
  uint16_t readRequestID(const uint8_t*& data_ptr) const;
  uint8_t readHubCntr(const uint8_t*& data_ptr) const;
  uint8_t readNoC(const uint8_t*& data_ptr) const;
  uint32_t readSessionID(const uint8_t*& data_ptr) const;
  uint8_t readCommandType(const uint8_t*& data_ptr) const;
  uint8_t readCommandMode(const uint8_t*& data_ptr) const;
  uint16_t readErrorCode(const uint8_t*& data_ptr) const;
  void readData(const datastructure::PacketBuffer& buffer, std::vector<uint8_t>& byteVector) const;
  void setCommandValuesFromPacket(const sick::datastructure::PacketBuffer& buffer,
                                  sick::cola2::Command& command) const;
};

} // namespace data_processing
} // namespace sick

#endif // SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_PARSETCPPACKET_H
