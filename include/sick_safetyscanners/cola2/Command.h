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
 * \file Command.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_COLA2_COMMAND_H
#define SICK_SAFETYSCANNERS_COLA2_COMMAND_H

#include <ros/ros.h>

#include <vector>

#include <sick_safetyscanners/datastructure/PacketBuffer.h>

#include <sick_safetyscanners/data_processing/ParseTCPPacket.h>
#include <sick_safetyscanners/data_processing/ReadWriteHelper.hpp>

#include <boost/thread/mutex.hpp>

namespace sick {
namespace cola2 {

/*!
 * \brief Forward declaration of the cola2session class.
 */
class Cola2Session;


/*!
 * \brief Base class for commands. Defines the base interface and does the common tasks.
 */
class Command
{
public:
  /*!
   * \brief Constructor of the command. Sets the common variables for a command to the sensor.
   *
   * \param session The session in which the command will be executed.
   * \param command_type Defines what type of command will be executed in the sensor (Read, Write,
   * Invoking a method).
   * \param command_mode Specifies the mode of the command. If the request is by index or name.
   */
  Command(sick::cola2::Cola2Session& session,
          const uint16_t& command_type,
          const uint16_t& command_mode);

  /*!
   * \brief We have virtual member functions, so a virtual destructor is needed.
   */
  virtual ~Command() {}


  /*!
   * \brief Locks a mutex to prevent other commands being executed in parallel.
   */
  void lockExecutionMutex();

  /*!
   * \brief Adds the data to the telegram and afterwards the header with the correct length.
   *
   * \param telegram The telegram, which will be modified with the data and header.
   * \returns Completed telegram.
   */
  std::vector<uint8_t> constructTelegram(const std::vector<uint8_t>& telegram) const;

  /*!
   * \brief Parses the da incoming data package and then processes it with the inherited
   * processReply. Afterwards the mutex will be unlocked to allow new commands to be send.
   *
   *
   * \param packet The incoming data package which will be processed.
   */
  void processReplyBase(const std::vector<uint8_t>& packet);


  /*!
   * \brief Scooped call to the mutex, which will block until the reply was processed.
   */
  void waitForCompletion();


  /*!
   * \brief Returns the current session ID.
   *
   * \returns The current session ID.
   */
  uint32_t getSessionID() const;


  /*!
   * \brief Sets the session ID.
   *
   * \param session_id The new session ID.
   */
  void setSessionID(const uint32_t& session_id);


  /*!
   * \brief Returns if the command was successfully parsed.
   *
   * \returns If the command was successfully parsed.
   */
  bool wasSuccessful() const;


  /*!
   * \brief Returns the command type.
   *
   * \returns The command type.
   */
  uint8_t getCommandType() const;

  /*!
   * \brief Sets the command type.
   *
   * \param command_type The new command type.
   */
  void setCommandType(const uint8_t& command_type);

  /*!
   * \brief Returns the command mode.
   *
   * \returns The command mode.
   */
  uint8_t getCommandMode() const;

  /*!
   * \brief Sets the command mode.
   *
   * \param command_mode The new command mode.
   */
  void setCommandMode(const uint8_t& command_mode);

  /*!
   * \brief Returns the request id of the command.
   *
   * \returns The request id.
   */
  uint16_t getRequestID() const;

  /*!
   * \brief Sets the request ID of the command.
   *
   * \param request_id The new request id.
   */
  void setRequestID(const uint16_t& request_id);

  /*!
   * \brief Returns the data vector.
   *
   * \returns The data vector.
   */
  std::vector<uint8_t> getDataVector() const;

  /*!
   * \brief Sets the data vector.
   *
   * \param data The new data vector.
   */
  void setDataVector(const std::vector<uint8_t>& data);

protected:
  sick::cola2::Cola2Session& m_session;

  std::vector<uint8_t> expandTelegram(const std::vector<uint8_t>& telegram,
                                      size_t additional_bytes) const;

private:
  std::shared_ptr<sick::data_processing::ParseTCPPacket> m_tcp_parser_ptr;

  boost::mutex m_execution_mutex;

  bool m_was_successful;

  uint8_t m_command_mode;
  uint8_t m_command_type;

  uint32_t m_session_id;
  uint16_t m_request_id;

  std::vector<uint8_t> m_data_vector;

  virtual bool processReply()                                                              = 0;
  virtual std::vector<uint8_t> addTelegramData(const std::vector<uint8_t>& telegram) const = 0;

  std::vector<uint8_t> addTelegramHeader(const std::vector<uint8_t>& telegram) const;
  std::vector<uint8_t> prepareHeader() const;
  void writeCola2StxToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeLengthToDataPtr(std::vector<uint8_t>::iterator data_ptr,
                            const std::vector<uint8_t>& telegram) const;
  void writeCola2HubCntrToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeCola2NoCToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeSessionIdToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeRequestIdToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeCommandTypeToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeCommandModeToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeDataToDataPtr(std::vector<uint8_t>::iterator data_ptr,
                          const std::vector<uint8_t>& telegram) const;
};


} // namespace cola2
} // namespace sick


#endif // SICK_SAFETYSCANNERS_COLA2_COMMAND_H
