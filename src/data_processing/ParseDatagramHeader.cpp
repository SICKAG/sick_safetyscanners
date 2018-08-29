#include <sick_microscan3_ros_driver/data_processing/ParseDatagramHeader.h>

namespace sick {
namespace data_processing {

bool ParseDatagramHeader::parseUDPSequence(datastructure::PaketBuffer buffer, datastructure::DatagramHeader &header)
{
  std::cout << "Beginn Parsing Header" << std::endl;

  const BYTE* dataPtr(buffer.getBuffer().data());

  header.setDatagramMarker(sick::data_processing::ReadWriteHelper::readUINT32BE(dataPtr));
  std::cout << header.getDatagramMarker() << std::endl;

  header.setProtocol(sick::data_processing::ReadWriteHelper::readUINT16BE(dataPtr));
  std::cout << header.getProtocol() << std::endl;

  header.setMajorVersion(sick::data_processing::ReadWriteHelper::readUINT8LE(dataPtr));
  std::cout << (unsigned)header.getMajorVersion() << std::endl; //Single byte can't be printed

  header.setMinorVersion(sick::data_processing::ReadWriteHelper::readUINT8LE(dataPtr));
  std::cout << (unsigned)header.getMinorVersion() << std::endl; //Single byte can't be printed

  header.setTotalLength(sick::data_processing::ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << header.getTotalLength() << std::endl;

  header.setIdentification(sick::data_processing::ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << header.getIdentification() << std::endl;

  header.setFragmentOffset(sick::data_processing::ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << header.getFragmentOffset() << std::endl;

  return true;
}

}
}

