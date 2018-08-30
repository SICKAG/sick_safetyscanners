#include <sick_microscan3_ros_driver/data_processing/ParseDerivedValues.h>

namespace sick {
namespace data_processing {

bool ParseDerivedValues::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::DerivedValues &data)
{
  std::cout << "Beginn Parsing DerivedValues" << std::endl;

  const BYTE* dataPtr(buffer.getBuffer().data());



  return true;
}

}
}

