#pragma once

#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>
#include <sick_microscan3_ros_driver/datastructure/Data.h>
#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>

namespace sick {
namespace data_processing {

class AbstractParseUDPSequence
{
public:
  //TODO add Template second argument or something similar
  static bool parseUDPSequence(sick::datastructure::PacketBuffer buffer);

private:
  AbstractParseUDPSequence();
};

}
}


