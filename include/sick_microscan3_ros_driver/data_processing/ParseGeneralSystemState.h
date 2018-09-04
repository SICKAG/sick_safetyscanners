#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>


namespace sick {
namespace data_processing {

class ParseGeneralSystemState : public AbstractParseUDPSequence
{
public:
  static datastructure::GeneralSystemState parseUDPSequence(sick::datastructure::PacketBuffer buffer, datastructure::Data &data);

private:
  ParseGeneralSystemState();
};

}
}


