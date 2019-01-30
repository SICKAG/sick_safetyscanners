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
 * \file CreateSession.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_COLA2_CREATESESSION_H
#define SICK_SAFETYSCANNERS_COLA2_CREATESESSION_H


#include <sick_safetyscanners/cola2/Command.h>

namespace sick {
namespace cola2 {

/*!
 * \brief Command to create a new cola2 session.
 */
class CreateSession : public Command
{
public:
  /*!
   * \brief Constructor to create a new command to set up a new session.
   *
   * \param session The new session which will be setup.
   */
  explicit CreateSession(Cola2Session& session);

  /*!
   * \brief Adds the data to the telegram.
   *
   * \param telegram The telegram which will be modified by the data.
   * \returns Completed new telegram message
   */
  std::vector<uint8_t> addTelegramData(const std::vector<uint8_t>& telegram) const;

  /*!
   * \brief Returns true since creating a new session is possible without a session ID.
   *
   * \returns true.
   */
  bool canBeExecutedWithoutSessionID() const;

  /*!
   * \brief Processes the return from the sensor. Checks if the request was successful.
   *
   * \returns If processing of the returned data was successful.
   */
  bool processReply();

private:
  /*std::vector<uint8_t>
  prepareTelegramAndGetDataPtr(const std::vector<uint8_t>& telegram) const;*/
  void writeHeartbeatTimeoutToDataPtr(std::vector<uint8_t>::iterator it) const;
  void writeClientIdToDataPtr(std::vector<uint8_t>::iterator it) const;
};

} // namespace cola2
} // namespace sick

#endif // SICK_SAFETYSCANNERS_COLA2_CREATESESSION_H
