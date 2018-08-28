#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/DatagramHeader.h>

namespace sick {
namespace data_processing {

class ParseDatagramHeader : public AbstractParseUDPSequence
{
public:
  static bool parseUDPSequence(sick::datastructure::PaketBuffer buffer, sick::datastructure::DatagramHeader header);

private:
  ParseDatagramHeader();
};

}
}


