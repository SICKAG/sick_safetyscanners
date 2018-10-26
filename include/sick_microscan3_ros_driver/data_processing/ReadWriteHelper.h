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

#ifndef SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_READWRITEHELPER_H
#define SICK_MICROSCAN3_ROS_DRIVER_DATA_PROCESSING_READWRITEHELPER_H

#include <stdint.h>

namespace sick {
namespace data_processing {

class ReadWriteHelper
{
public:
  ReadWriteHelper();

  void writeuint8_t(uint8_t*& buf, const uint8_t v, const uint16_t offset) const;
  void writeuint8_tBigEndian(uint8_t*& buf, const uint8_t v, const uint16_t offset) const;
  void writeuint8_tLittleEndian(uint8_t*& buf, const uint8_t v, const uint16_t offset) const;
  void writeint8_t(uint8_t*& buf, const uint8_t v, const uint16_t offset) const;
  void writeint8_tBigEndian(uint8_t*& buf, const uint8_t v, const uint16_t offset) const;
  void writeint8_tLittleEndian(uint8_t*& buf, const uint8_t v, const uint16_t offset) const;

  void writeuint16_tBigEndian(uint8_t*& buf, const uint16_t v, const uint16_t offset) const;
  void writeuint16_tLittleEndian(uint8_t*& buf, const uint16_t v, const uint16_t offset) const;

  void writeuint32_tBigEndian(uint8_t*& buf, const uint32_t v, const uint16_t offset) const;
  void writeuint32_tLittleEndian(uint8_t*& buf, const uint32_t v, const uint16_t offset) const;

  uint8_t readuint8_t(const uint8_t*& buf, const uint16_t offset) const;
  uint8_t readuint8_tBigEndian(const uint8_t*& buf, const uint16_t offset) const;
  uint8_t readuint8_tLittleEndian(const uint8_t*& buf, const uint16_t offset) const;
  int8_t readint8_t(const uint8_t*& buffer, const uint16_t offset) const;
  int8_t readint8_tBigEndian(const uint8_t*& buf, const uint16_t offset) const;
  int8_t readint8_tLittleEndian(const uint8_t*& buf, const uint16_t offset) const;

  uint16_t readuint16_tBigEndian(const uint8_t*& buf, const uint16_t offset) const;
  uint16_t readuint16_tLittleEndian(const uint8_t*& buf, const uint16_t offset) const;
  int16_t readint16_tBigEndian(const uint8_t*& buf, const uint16_t offset) const;
  int16_t readint16_tLittleEndian(const uint8_t*& buf, const uint16_t offset) const;

  uint32_t readuint32_tBigEndian(const uint8_t*& buf, const uint16_t offset) const;
  uint32_t readuint32_tLittleEndian(const uint8_t*& buf, const uint16_t offset) const;
  int32_t readint32_tBigEndian(const uint8_t*& buf, const uint16_t offset) const;
  int32_t readint32_tLittleEndian(const uint8_t*& buf, const uint16_t offset) const;
};

}  // namespace data_processing
}  // namespace sick

#endif
