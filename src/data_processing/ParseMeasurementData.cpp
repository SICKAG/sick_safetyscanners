#include <sick_microscan3_ros_driver/data_processing/ParseMeasurementData.h>

namespace sick {
namespace data_processing {

bool ParseMeasurementData::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::MeasurementData &data)
{
  std::cout << "Beginn Parsing Header" << std::endl;

  const BYTE* dataPtr(buffer.getBuffer().data());



  return true;
}

}
}

