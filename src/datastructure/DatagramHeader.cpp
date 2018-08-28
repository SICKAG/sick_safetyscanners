#include <sick_microscan3_ros_driver/datastructure/DatagramHeader.h>

namespace sick {
namespace datastructure {

DatagramHeader::DatagramHeader()
{

}

UINT32 DatagramHeader::getDatagramMarker() const
{
  return m_datagram_marker;
}

void DatagramHeader::setDatagramMarker(const UINT32 &value)
{
  m_datagram_marker = value;
}

UINT16 DatagramHeader::getProtocol() const
{
  return m_protocol;
}

void DatagramHeader::setProtocol(const UINT16 &value)
{
  m_protocol = value;
}

UINT8 DatagramHeader::getMajorVersion() const
{
  return m_major_version;
}

void DatagramHeader::setMajorVersion(const UINT8 &value)
{
  m_major_version = value;
}

UINT8 DatagramHeader::getMinorVersion() const
{
  return m_minor_version;
}

void DatagramHeader::setMinorVersion(const UINT8 &value)
{
  m_minor_version = value;
}

UINT32 DatagramHeader::getTotalLength() const
{
  return m_total_length;
}

void DatagramHeader::setTotalLength(const UINT32 &value)
{
  m_total_length = value;
}

UINT32 DatagramHeader::getIdentification() const
{
  return m_identification;
}

void DatagramHeader::setIdentification(const UINT32 &value)
{
  m_identification = value;
}

UINT32 DatagramHeader::getFragmentOffset() const
{
  return m_fragment_offset;
}

void DatagramHeader::setFragmentOffset(const UINT32 &value)
{
  m_fragment_offset = value;
}

}
}

