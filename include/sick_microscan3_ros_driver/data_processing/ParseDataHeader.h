#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/DataHeader.h>


namespace sick {
namespace data_processing {

class ParseDataHeader : public AbstractParseUDPSequence
{
public:
  static bool parseUDPSequence(sick::datastructure::PacketBuffer buffer, sick::datastructure::DataHeader& data);

private:
  ParseDataHeader();
};

}
}


