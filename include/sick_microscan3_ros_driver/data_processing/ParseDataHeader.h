#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/DataHeader.h>


namespace sick {
namespace data_processing {

class ParseDataHeader : public AbstractParseUDPSequence
{
public:
  ParseDataHeader();

  datastructure::DataHeader parseUDPSequence(sick::datastructure::PacketBuffer buffer, datastructure::Data &data);

private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
};

}
}


