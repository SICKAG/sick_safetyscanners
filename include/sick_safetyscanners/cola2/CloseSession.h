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
 * \file CloseSession.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_COLA2_CLOSESESSION_H
#define SICK_SAFETYSCANNERS_COLA2_CLOSESESSION_H


#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

/*!
 * \brief Command to close a cola2 session between host and sensor.
 */
class CloseSession : public Command
{
public:
  /*!
   * \brief Constructor of the command to close a cola2 session.
   *
   * \param session The currents cola2 session, which will be closed on executing the command.
   */
  explicit CloseSession(Cola2Session& session);

  /*!
   * \brief Adds data to the telegram. The close cola2 session command does not carry any extra data
   * and therefor this function does not add any data.
   *
   * \param telegram The telegram which will be modified.
   & \returns Completed telegram.
   */
  std::vector<uint8_t> addTelegramData(const std::vector<uint8_t>& telegram) const;


  /*!
   * \brief Returns if the command can be executed without a session ID. Will return false for most
   * commands except the commands to establish a connection.
   *
   * \returns If the command needs a session ID to be executed.
   */
  bool canBeExecutedWithoutSessionID() const;

  /*!
   * \brief Processes the return from the sensor.
   *
   * \returns If processing of the returned data was successful.
   */
  bool processReply();
};

} // namespace cola2
} // namespace sick

#endif // SICK_SAFETYSCANNERS_COLA2_CLOSESESSION_H
