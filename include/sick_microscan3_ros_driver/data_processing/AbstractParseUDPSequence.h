#pragma once

#include <sick_microscan3_ros_driver/datastructure/PaketBuffer.h>

namespace sick {
namespace data_processing {

class AbstractParseUDPSequence
{
public:
  static bool parseUDPSequence(sick::datastructure::PaketBuffer buffer);

private:
  AbstractParseUDPSequence();
};

}
}


