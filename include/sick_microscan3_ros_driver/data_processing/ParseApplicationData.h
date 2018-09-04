#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>


namespace sick {
namespace data_processing {

class ParseApplicationData : public AbstractParseUDPSequence
{
public:
  static datastructure::ApplicationData parseUDPSequence(sick::datastructure::PacketBuffer buffer, datastructure::Data &data);

private:
  ParseApplicationData();
};

}
}


