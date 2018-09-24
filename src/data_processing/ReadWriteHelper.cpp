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

#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>

namespace sick {
namespace data_processing {

ReadWriteHelper::ReadWriteHelper()
{

}

//*************
// UINT 8

UINT8 ReadWriteHelper::readUINT8(const BYTE *&buffer, UINT16 offset) {
  UINT8 value = buffer[offset];
  return value;
}

UINT8 ReadWriteHelper::readUINT8LittleEndian(const BYTE *&buf, UINT16 offset) {
  return readUINT8(buf, offset);
}

UINT8 ReadWriteHelper::readUINT8BigEndian(const BYTE *&buf, UINT16 offset) {
  return readUINT8(buf, offset);
}

void ReadWriteHelper::writeUINT8 (BYTE* &buf, UINT8 v, UINT16 offset)
{
  buf[offset] = v;
}

void ReadWriteHelper::writeUINT8BigEndian (BYTE* &buf, UINT8 v, UINT16 offset)
{
  writeUINT8(buf, v, offset);
}

void ReadWriteHelper::writeUINT8LittleEndian (BYTE* &buf, UINT8 v, UINT16 offset)
{
  writeUINT8(buf, v, offset);
}


//*******
// INT8

INT8 ReadWriteHelper::readINT8(const BYTE *& buffer, UINT16 offset) {
  return readUINT8(buffer, offset);
}


INT8 ReadWriteHelper::readINT8LittleEndian(const BYTE *&buf, UINT16 offset) {
  return readINT8(buf, offset);
}

INT8 ReadWriteHelper::readINT8BigEndian(const BYTE *&buf, UINT16 offset) {
  return readINT8(buf, offset);
}

void ReadWriteHelper::writeINT8 (BYTE* &buf, UINT8 v, UINT16 offset)
{
  writeUINT8(buf,v,offset);
}

void ReadWriteHelper::writeINT8BigEndian (BYTE* &buf, UINT8 v, UINT16 offset)
{
  writeINT8(buf, v, offset);
}

void ReadWriteHelper::writeINT8LittleEndian (BYTE* &buf, UINT8 v, UINT16 offset)
{
  writeINT8(buf, v, offset);
}

//*******
//  UINT16

UINT16 ReadWriteHelper::readUINT16LittleEndian(const BYTE *&buf, UINT16 offset) {
  return (buf[offset + 1] << 8)
      + buf[offset];
}

UINT16 ReadWriteHelper::readUINT16BigEndian(const BYTE *&buf, UINT16 offset) {
  return (buf[offset] << 8)
      + buf[offset + 1];
}

void ReadWriteHelper::writeUINT16BigEndian (BYTE* &buf, UINT16 v, UINT16 offset)
{
  buf[offset]     = (v & 0xff00) >> 8;
  buf[offset + 1] = v & 0xff;
}

void ReadWriteHelper::writeUINT16LittleEndian (BYTE* &buf, UINT16 v, UINT16 offset)
{
  buf[offset + 1] = (v & 0xff00) >> 8;
  buf[offset]     = v & 0xff;
}

//*******
// INT16

INT16 ReadWriteHelper::readINT16LittleEndian(const BYTE *&buf, UINT16 offset) {
  return readUINT16LittleEndian(buf, offset);
}

INT16 ReadWriteHelper::readINT16BigEndian(const BYTE *&buf, UINT16 offset) {
  return readUINT16BigEndian(buf, offset);
}


//*******
// UINT32


UINT32 ReadWriteHelper::readUINT32LittleEndian(const BYTE *&buf, UINT16 offset) {
  return (buf[offset + 3] << 24)
      + (buf[offset + 2] << 16)
      + (buf[offset + 1] << 8)
      + buf[offset];
}

UINT32 ReadWriteHelper::readUINT32BigEndian(const BYTE *&buf, UINT16 offset) {
  return (buf[offset] << 24)
      + (buf[offset + 1] << 16)
      + (buf[offset + 2] << 8)
      + buf[offset + 3];
}

void ReadWriteHelper::writeUINT32LittleEndian (BYTE* &buf, UINT32 v, UINT16 offset)
{
  buf[offset + 3] = (v & 0xff000000) >> 24;
  buf[offset + 2] = (v & 0xff0000) >> 16;
  buf[offset + 1] = (v & 0xff00) >> 8;
  buf[offset    ] = v & 0xff;
}

void ReadWriteHelper::writeUINT32BigEndian (BYTE* &buf, UINT32 v, UINT16 offset)
{
  buf[offset] = (v & 0xff000000) >> 24;
  buf[offset + 1] = (v & 0xff0000) >> 16;
  buf[offset + 2] = (v & 0xff00) >> 8;
  buf[offset + 3] = v & 0xff;
}


//*******
// INT32

INT32 ReadWriteHelper::readINT32LittleEndian(const BYTE *&buf, UINT16 offset) {
  return readUINT32LittleEndian(buf, offset);
}

INT32 ReadWriteHelper::readINT32BigEndian(const BYTE *&buf, UINT16 offset) {
  return readUINT32BigEndian(buf, offset);
}



}
}
