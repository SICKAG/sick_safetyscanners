#pragma once

#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>

namespace sick {
namespace data_processing {

class AbstractParseUDPSequence
{
public:
  static bool parseUDPSequence(sick::datastructure::PacketBuffer buffer);

private:
  AbstractParseUDPSequence();
};

}
}


