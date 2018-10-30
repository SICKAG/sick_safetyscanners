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
 * \file ReadWriteHelper.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners/data_processing/ReadWriteHelper.h>

namespace sick {
namespace data_processing {

ReadWriteHelper::ReadWriteHelper() {}

//*************
// UINT 8

uint8_t ReadWriteHelper::readuint8_t(const uint8_t*& buf, const uint16_t offset) const
{
  uint8_t value = buf[offset];
  return value;
}

uint8_t ReadWriteHelper::readuint8_tLittleEndian(const uint8_t*& buf, const uint16_t offset) const
{
  return readuint8_t(buf, offset);
}

uint8_t ReadWriteHelper::readuint8_tBigEndian(const uint8_t*& buf, const uint16_t offset) const
{
  return readuint8_t(buf, offset);
}

void ReadWriteHelper::writeuint8_t(uint8_t*& buf, const uint8_t v, const uint16_t offset) const
{
  buf[offset] = v;
}

void ReadWriteHelper::writeuint8_tBigEndian(uint8_t*& buf,
                                            const uint8_t v,
                                            const uint16_t offset) const
{
  writeuint8_t(buf, v, offset);
}

void ReadWriteHelper::writeuint8_tLittleEndian(uint8_t*& buf,
                                               const uint8_t v,
                                               const uint16_t offset) const
{
  writeuint8_t(buf, v, offset);
}


//*******
// int8_t

int8_t ReadWriteHelper::readint8_t(const uint8_t*& buffer, const uint16_t offset) const
{
  return readuint8_t(buffer, offset);
}


int8_t ReadWriteHelper::readint8_tLittleEndian(const uint8_t*& buf, const uint16_t offset) const
{
  return readint8_t(buf, offset);
}

int8_t ReadWriteHelper::readint8_tBigEndian(const uint8_t*& buf, const uint16_t offset) const
{
  return readint8_t(buf, offset);
}

void ReadWriteHelper::writeint8_t(uint8_t*& buf, const uint8_t v, const uint16_t offset) const
{
  writeuint8_t(buf, v, offset);
}

void ReadWriteHelper::writeint8_tBigEndian(uint8_t*& buf,
                                           const uint8_t v,
                                           const uint16_t offset) const
{
  writeint8_t(buf, v, offset);
}

void ReadWriteHelper::writeint8_tLittleEndian(uint8_t*& buf,
                                              const uint8_t v,
                                              const uint16_t offset) const
{
  writeint8_t(buf, v, offset);
}

//*******
//  uint16_t

uint16_t ReadWriteHelper::readuint16_tLittleEndian(const uint8_t*& buf, const uint16_t offset) const
{
  return (buf[offset + 1] << 8) + buf[offset];
}

uint16_t ReadWriteHelper::readuint16_tBigEndian(const uint8_t*& buf, const uint16_t offset) const
{
  return (buf[offset] << 8) + buf[offset + 1];
}

void ReadWriteHelper::writeuint16_tBigEndian(uint8_t*& buf,
                                             const uint16_t v,
                                             const uint16_t offset) const
{
  buf[offset]     = (v & 0xff00) >> 8;
  buf[offset + 1] = v & 0xff;
}

void ReadWriteHelper::writeuint16_tLittleEndian(uint8_t*& buf,
                                                const uint16_t v,
                                                const uint16_t offset) const
{
  buf[offset + 1] = (v & 0xff00) >> 8;
  buf[offset]     = v & 0xff;
}

//*******
// int16_t

int16_t ReadWriteHelper::readint16_tLittleEndian(const uint8_t*& buf, const uint16_t offset) const
{
  return readuint16_tLittleEndian(buf, offset);
}

int16_t ReadWriteHelper::readint16_tBigEndian(const uint8_t*& buf, const uint16_t offset) const
{
  return readuint16_tBigEndian(buf, offset);
}


//*******
// uint32_t


uint32_t ReadWriteHelper::readuint32_tLittleEndian(const uint8_t*& buf, const uint16_t offset) const
{
  return (buf[offset + 3] << 24) + (buf[offset + 2] << 16) + (buf[offset + 1] << 8) + buf[offset];
}

uint32_t ReadWriteHelper::readuint32_tBigEndian(const uint8_t*& buf, const uint16_t offset) const
{
  return (buf[offset] << 24) + (buf[offset + 1] << 16) + (buf[offset + 2] << 8) + buf[offset + 3];
}

void ReadWriteHelper::writeuint32_tLittleEndian(uint8_t*& buf,
                                                const uint32_t v,
                                                const uint16_t offset) const
{
  buf[offset + 3] = (v & 0xff000000) >> 24;
  buf[offset + 2] = (v & 0xff0000) >> 16;
  buf[offset + 1] = (v & 0xff00) >> 8;
  buf[offset]     = v & 0xff;
}

void ReadWriteHelper::writeuint32_tBigEndian(uint8_t*& buf,
                                             const uint32_t v,
                                             const uint16_t offset) const
{
  buf[offset]     = (v & 0xff000000) >> 24;
  buf[offset + 1] = (v & 0xff0000) >> 16;
  buf[offset + 2] = (v & 0xff00) >> 8;
  buf[offset + 3] = v & 0xff;
}


//*******
// int32_t

int32_t ReadWriteHelper::readint32_tLittleEndian(const uint8_t*& buf, const uint16_t offset) const
{
  return readuint32_tLittleEndian(buf, offset);
}

int32_t ReadWriteHelper::readint32_tBigEndian(const uint8_t*& buf, const uint16_t offset) const
{
  return readuint32_tBigEndian(buf, offset);
}


} // namespace data_processing
} // namespace sick
