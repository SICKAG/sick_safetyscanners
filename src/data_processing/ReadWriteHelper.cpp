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


UINT32 ReadWriteHelper::readUINT32LE(const BYTE *&buf)
{
  UINT32 value = (buf[3] << 24) + (buf[2] << 16) + (buf[1] << 8) + buf[0];
  buf += sizeof(UINT32);
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

}
}
