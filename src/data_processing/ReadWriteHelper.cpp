#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>

namespace sick {
namespace data_processing {

ReadWriteHelper::ReadWriteHelper()
{

}

UINT8 ReadWriteHelper::readUINT8LittleEndian(const BYTE *&buf)
{
  UINT8 value = buf[0];
  buf += sizeof(UINT8);
  return value;
}

UINT8 ReadWriteHelper::readUINT8LittleEndian(const BYTE *&buf, UINT16 offset) {
  UINT8 value = buf[offset];
  return value;
}

UINT16 ReadWriteHelper::readUINT16LittleEndian(const BYTE *&buf)
{
  UINT16 value = (buf[1] << 8) + buf[0];
  buf += sizeof(UINT16);
  return value;
}

UINT16 ReadWriteHelper::readUINT16LittleEndian(const BYTE *&buf, UINT16 offset) {
  UINT16 value = (buf[offset + 1] << 8) + buf[offset];
  return value;
}


INT16 ReadWriteHelper::readINT16LittleEndian(const BYTE *&buf)
{
  INT16 value = (buf[1] << 8) + buf[0];
  buf += sizeof(INT16);
  return value;
}


UINT32 ReadWriteHelper::readUINT32LittleEndian(const BYTE *&buf)
{
  UINT32 value = (buf[3] << 24) + (buf[2] << 16) + (buf[1] << 8) + buf[0];
  buf += sizeof(UINT32);
  return value;
}

UINT32 ReadWriteHelper::readUINT32LittleEndian(const BYTE *&buf, UINT16 offset) {
  UINT32 value = (buf[offset + 3] << 24) + (buf[offset + 2] << 16) + (buf[offset + 1] << 8) + buf[offset];
  return value;
}

INT32 ReadWriteHelper::readINT32LittleEndian(const BYTE *&buf)
{
  INT32 value = (buf[3] << 24) + (buf[2] << 16) + (buf[1] << 8) + buf[0];
  buf += sizeof(INT32);
  return value;
}

INT32 ReadWriteHelper::readINT32LittleEndian(const BYTE *&buf, UINT16 offset) {
  INT32 value = (buf[offset + 3] << 24) + (buf[offset + 2] << 16) + (buf[offset + 1] << 8) + buf[offset];
  return value;
}


UINT8 ReadWriteHelper::readUINT8BigEndian (const BYTE *&buf)
{
  UINT8 value = buf[0];
  buf += sizeof(UINT8);
  return value;
}

UINT16 ReadWriteHelper::readUINT16BigEndian (const BYTE *&buf)
{
  UINT16 value = (buf[0] << 8) + buf[1];
  buf += sizeof(UINT16);
  return value;
}

UINT32 ReadWriteHelper::readUINT32BigEndian (const BYTE *&buf)
{
  UINT32 value = (buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];
  buf += sizeof(UINT32);
  return value;
}


void ReadWriteHelper::writeUINT8BigEndian (BYTE* &buf, UINT8 v)
{
  buf[0] = v;
  buf += sizeof(UINT8);
}

void ReadWriteHelper::writeUINT16BigEndian (BYTE* &buf, UINT16 v)
{
  buf[0] = (v & 0xff00) >> 8;
  buf[1] = v & 0xff;
  buf += sizeof(UINT16);
}

void ReadWriteHelper::writeUINT32BigEndian (BYTE* &buf, UINT32 v)
{
  buf[0] = (v & 0xff000000) >> 24;
  buf[1] = (v & 0xff0000) >> 16;
  buf[2] = (v & 0xff00) >> 8;
  buf[3] = v & 0xff;
  buf += sizeof(UINT32);
}

void ReadWriteHelper::writeUINT8LittleEndian (BYTE* &buf, UINT8 v)
{
  buf[0] = v;
  buf += sizeof(UINT8);
}

void ReadWriteHelper::writeUINT16LittleEndian (BYTE* &buf, UINT16 v)
{
  buf[1] = (v & 0xff00) >> 8;
  buf[0] = v & 0xff;
  buf += sizeof(UINT16);
}

void ReadWriteHelper::writeUINT32LittleEndian (BYTE* &buf, UINT32 v)
{
  buf[3] = (v & 0xff000000) >> 24;
  buf[2] = (v & 0xff0000) >> 16;
  buf[1] = (v & 0xff00) >> 8;
  buf[0] = v & 0xff;
  buf += sizeof(UINT32);
}

}
}
