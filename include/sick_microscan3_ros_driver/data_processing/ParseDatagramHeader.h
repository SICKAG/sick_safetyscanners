#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/DatagramHeader.h>
#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>

namespace sick {
namespace data_processing {

class ParseDatagramHeader : public AbstractParseUDPSequence
{
public:
  ParseDatagramHeader();
  bool parseUDPSequence(sick::datastructure::PacketBuffer buffer, sick::datastructure::DatagramHeader& header);

private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;

  bool setDataInHeader(const BYTE *data_ptr, datastructure::DatagramHeader &header);

  bool setDatagramMarkerInHeader(const BYTE *data_ptr, datastructure::DatagramHeader &header);
  bool setProtocolInHeader(const BYTE *data_ptr, datastructure::DatagramHeader &header);
  bool setMajorVersionInHeader(const BYTE *data_ptr, datastructure::DatagramHeader &header);
  bool setMinorVersionInHeader(const BYTE *data_ptr, datastructure::DatagramHeader &header);
  bool setTotalLengthInHeader(const BYTE *data_ptr, datastructure::DatagramHeader &header);
  bool setIdentificationInHeader(const BYTE *data_ptr, datastructure::DatagramHeader &header);
  bool setFragmentOffsetInHeader(const BYTE *data_ptr, datastructure::DatagramHeader &header);

};

}
}


