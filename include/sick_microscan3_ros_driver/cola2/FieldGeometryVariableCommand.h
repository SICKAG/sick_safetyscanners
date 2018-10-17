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
 * \file FieldGeometryVariableCommand.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-10-16
 */
//----------------------------------------------------------------------

#ifndef FIELDGEOMETRYVARIABLECOMMAND_H
#define FIELDGEOMETRYVARIABLECOMMAND_H


#include <sick_microscan3_ros_driver/cola2/VariableCommand.h>
#include <sick_microscan3_ros_driver/datastructure/CommSettings.h>
#include <sick_microscan3_ros_driver/data_processing/ParseTypeCodeData.h>

namespace sick {
namespace cola2 {

class FieldGeometryVariableCommand : public VariableCommand
{
public:
  typedef sick::cola2::VariableCommand base_class;

  FieldGeometryVariableCommand(Cola2Session& session, datastructure::TypeCode &type_code);
  void addTelegramData(sick::datastructure::PacketBuffer::VectorBuffer& telegram) const;
  bool canBeExecutedWithoutSessionID() const;
  bool processReply();


private:
  std::shared_ptr<sick::data_processing::ReadWriteHelper> m_writer_ptr;
  std::shared_ptr<sick::data_processing::ParseTypeCodeData> m_type_code_parser_ptr;

  sick::datastructure::TypeCode& m_type_code;

};

} // namespace cola2
} // namespace sick

#endif
