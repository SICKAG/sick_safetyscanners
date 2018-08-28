#pragma once

#include <sick_microscan3_ros_driver/datastructure/DataTypes.h>

namespace sick {
namespace datastructure {

class DatagramHeader
{
public:
  DatagramHeader();

  UINT32 getDatagramMarker() const;
  void setDatagramMarker(const UINT32 &value);

  UINT16 getProtocol() const;
  void setProtocol(const UINT16 &value);

  UINT8 getMajorVersion() const;
  void setMajorVersion(const UINT8 &value);

  UINT8 getMinorVersion() const;
  void setMinorVersion(const UINT8 &value);

  UINT32 getTotalLength() const;
  void setTotalLength(const UINT32 &value);

  UINT32 getIdentification() const;
  void setIdentification(const UINT32 &value);

  UINT32 getFragmentOffset() const;
  void setFragmentOffset(const UINT32 &value);

private:
  UINT32 m_datagram_marker;
  UINT16 m_protocol;
  UINT8 m_major_version;
  UINT8 m_minor_version;
  UINT32 m_total_length;
  UINT32 m_identification;
  UINT32 m_fragment_offset;
};

}
}
