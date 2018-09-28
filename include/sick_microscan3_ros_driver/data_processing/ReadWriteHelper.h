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
 * \file ReadWriteHelper.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#ifndef READWRITEHELPER_H
#define READWRITEHELPER_H

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>

namespace sick {
namespace data_processing {

class ReadWriteHelper
{
public:
  ReadWriteHelper();

  void writeUINT8BigEndian(BYTE*& buf, UINT8 v, UINT16 offset);
  void writeUINT16BigEndian(BYTE*& buf, UINT16 v, UINT16 offset);
  void writeUINT32BigEndian(BYTE*& buf, UINT32 v, UINT16 offset);
  void writeUINT8LittleEndian(BYTE*& buf, UINT8 v, UINT16 offset);
  void writeUINT16LittleEndian(BYTE*& buf, UINT16 v, UINT16 offset);
  void writeUINT32LittleEndian(BYTE*& buf, UINT32 v, UINT16 offset);
  UINT8 readUINT8LittleEndian(const BYTE*& buf, UINT16 offset);
  UINT16 readUINT16LittleEndian(const BYTE*& buf, UINT16 offset);
  UINT32 readUINT32LittleEndian(const BYTE*& buf, UINT16 offset);
  INT32 readINT32LittleEndian(const BYTE*& buf, UINT16 offset);
  UINT8 readUINT8(const BYTE*& buffer, UINT16 offset);
  UINT8 readUINT8BigEndian(const BYTE*& buf, UINT16 offset);
  UINT16 readUINT16BigEndian(const BYTE*& buf, UINT16 offset);
  UINT32 readUINT32BigEndian(const BYTE*& buf, UINT16 offset);
  INT8 readINT8(const BYTE*& buffer, UINT16 offset);
  INT8 readINT8LittleEndian(const BYTE*& buf, UINT16 offset);
  INT8 readINT8BigEndian(const BYTE*& buf, UINT16 offset);
  INT16 readINT16LittleEndian(const BYTE*& buf, UINT16 offset);
  INT16 readINT16BigEndian(const BYTE*& buf, UINT16 offset);
  INT32 readINT32BigEndian(const BYTE*& buf, UINT16 offset);
  void writeUINT8(BYTE*& buf, UINT8 v, UINT16 offset);
  void writeINT8(BYTE*& buf, UINT8 v, UINT16 offset);
  void writeINT8BigEndian(BYTE*& buf, UINT8 v, UINT16 offset);
  void writeINT8LittleEndian(BYTE*& buf, UINT8 v, UINT16 offset);
};

} // namespace data_processing
} // namespace sick

#endif
