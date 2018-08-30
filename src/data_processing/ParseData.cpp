#include <sick_microscan3_ros_driver/data_processing/ParseData.h>

namespace sick {
namespace data_processing {

bool ParseData::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{
  std::cout << "Beginn Parsing Data Header" << std::endl;

  const BYTE* dataPtr(buffer.getBuffer().data());

  sick::datastructure::DataHeader data_header;
  ParseDataHeader::parseUDPSequence(buffer, data_header);
  data.setDataHeaderPtr(boost::make_shared<sick::datastructure::DataHeader>(data_header));

  std::cout << "DATA: scanNumber:  " << data.getDataHeaderPtr()->getScanNumber() << std::endl;



  return true;
}

}
}

