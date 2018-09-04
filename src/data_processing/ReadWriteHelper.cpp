#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>

namespace sick {
namespace data_processing {

ReadWriteHelper::ReadWriteHelper()
{

}

UINT8 ReadWriteHelper::readUINT8LE(const BYTE *&buf)
{
  UINT8 value = buf[0];
  buf += sizeof(UINT8);
  return value;
}

UINT16 ReadWriteHelper::readUINT16LE(const BYTE *&buf)
{
  UINT16 value = (buf[1] << 8) + buf[0];
  buf += sizeof(UINT16);
  return value;
}


INT16 ReadWriteHelper::readINT16LE(const BYTE *&buf)
{
  INT16 value = (buf[1] << 8) + buf[0];
  buf += sizeof(INT16);
  return value;
}


UINT32 ReadWriteHelper::readUINT32LE(const BYTE *&buf)
{
  UINT32 value = (buf[3] << 24) + (buf[2] << 16) + (buf[1] << 8) + buf[0];
  buf += sizeof(UINT32);
  return value;
}

INT32 ReadWriteHelper::readINT32LE(const BYTE *&buf)
{
  INT32 value = (buf[3] << 24) + (buf[2] << 16) + (buf[1] << 8) + buf[0];
  buf += sizeof(INT32);
  return value;
}



UINT64 ReadWriteHelper::readUINT64LE(const BYTE *&buf)
{
  UINT64 value = (UINT64(buf[7]) << 56) + (UINT64(buf[6]) << 48)
      + (UINT64(buf[5]) << 40) + (UINT64(buf[4]) << 32)
      + (UINT64(buf[3]) << 24) + (UINT64(buf[2]) << 16)
      + (UINT64(buf[1]) <<  8) +  UINT64(buf[0]);
  buf += sizeof(UINT64);
  return value;
}


UINT8 ReadWriteHelper::readUINT8BE (const BYTE *&buf)
{
  UINT8 value = buf[0];
  buf += sizeof(UINT8);
  return value;
}

UINT16 ReadWriteHelper::readUINT16BE (const BYTE *&buf)
{
  UINT16 value = (buf[0] << 8) + buf[1];
  buf += sizeof(UINT16);
  return value;
}

UINT32 ReadWriteHelper::readUINT32BE (const BYTE *&buf)
{
  UINT32 value = (buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];
  buf += sizeof(UINT32);
  return value;
}

UINT64 ReadWriteHelper::readUINT64BE (const BYTE *&buf)
{
  UINT64 value = (UINT64(buf[0]) << 56) + (UINT64(buf[1]) << 48)
      + (UINT64(buf[2]) << 40) + (UINT64(buf[3]) << 32)
      + (UINT64(buf[4]) << 24) + (UINT64(buf[5]) << 16)
      + (UINT64(buf[6]) <<  8) +  UINT64(buf[7]);
  buf += sizeof(UINT64);
  return value;
}


void ReadWriteHelper::writeUINT8BE (BYTE* buf, UINT8 v)
{
  buf[0] = v;
}

void ReadWriteHelper::writeUINT16BE (BYTE* buf, UINT16 v)
{
  buf[0] = (v & 0xff00) >> 8;
  buf[1] = v & 0xff;
}

void ReadWriteHelper::writeUINT32BE (BYTE* buf, UINT32 v)
{
  buf[0] = (v & 0xff000000) >> 24;
  buf[1] = (v & 0xff0000) >> 16;
  buf[2] = (v & 0xff00) >> 8;
  buf[3] = v & 0xff;
}

void ReadWriteHelper::writeUINT8LE (BYTE* buf, UINT8 v)
{
  buf[0] = v;
}

void ReadWriteHelper::writeUINT16LE (BYTE* buf, UINT16 v)
{
  buf[1] = (v & 0xff00) >> 8;
  buf[0] = v & 0xff;
}

void ReadWriteHelper::writeUINT32LE (BYTE* buf, UINT32 v)
{
  buf[3] = (v & 0xff000000) >> 24;
  buf[2] = (v & 0xff0000) >> 16;
  buf[1] = (v & 0xff00) >> 8;
  buf[0] = v & 0xff;
}

}
}
