#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/MeasurementData.h>


namespace sick {
namespace data_processing {

class ParseMeasurementData : public AbstractParseUDPSequence
{
public:
  static datastructure::MeasurementData parseUDPSequence(sick::datastructure::PacketBuffer buffer, datastructure::Data &header);

private:
  ParseMeasurementData();
};

}
}


