#include <sick_microscan3_ros_driver/data_processing/ParseDatagramHeader.h>

namespace sick {
namespace data_processing {

ParseDatagramHeader::ParseDatagramHeader()
{
  m_readerPtr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

bool ParseDatagramHeader::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::DatagramHeader &header)
{
  std::cout << "Beginn Parsing Header" << std::endl;

  const BYTE* dataPtr(buffer.getBuffer().data());

  header.setDatagramMarker(m_readerPtr->readUINT32BE(dataPtr));
//  std::cout << header.getDatagramMarker() << std::endl;

  header.setProtocol(m_readerPtr->readUINT16BE(dataPtr));
//  std::cout << header.getProtocol() << std::endl;

  header.setMajorVersion(m_readerPtr->readUINT8LE(dataPtr));
//  std::cout << (unsigned)header.getMajorVersion() << std::endl; //Single byte can't be printed

  header.setMinorVersion(m_readerPtr->readUINT8LE(dataPtr));
//  std::cout << (unsigned)header.getMinorVersion() << std::endl; //Single byte can't be printed

  header.setTotalLength(m_readerPtr->readUINT32LE(dataPtr));
//  std::cout << header.getTotalLength() << std::endl;

  header.setIdentification(m_readerPtr->readUINT32LE(dataPtr));
//  std::cout << header.getIdentification() << std::endl;

  header.setFragmentOffset(m_readerPtr->readUINT32LE(dataPtr));
//  std::cout << header.getFragmentOffset() << std::endl;

  return true;
}

}
}

