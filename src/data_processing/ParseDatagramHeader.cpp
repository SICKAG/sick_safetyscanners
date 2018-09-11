#include <sick_microscan3_ros_driver/data_processing/ParseDatagramHeader.h>

namespace sick {
namespace data_processing {

ParseDatagramHeader::ParseDatagramHeader()
{
  m_reader_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

bool ParseDatagramHeader::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::DatagramHeader &header)
{
  std::cout << "Beginn Parsing Header" << std::endl;

  const BYTE* data_ptr(buffer.getBuffer().data());

  header.setDatagramMarker(m_reader_ptr->readUINT32BigEndian(data_ptr, 0));
//  std::cout << header.getDatagramMarker() << std::endl;

  header.setProtocol(m_reader_ptr->readUINT16BigEndian(data_ptr,4));
//  std::cout << header.getProtocol() << std::endl;

  header.setMajorVersion(m_reader_ptr->readUINT8LittleEndian(data_ptr,6));
//  std::cout << (unsigned)header.getMajorVersion() << std::endl; //Single byte can't be printed

  header.setMinorVersion(m_reader_ptr->readUINT8LittleEndian(data_ptr,7));
//  std::cout << (unsigned)header.getMinorVersion() << std::endl; //Single byte can't be printed

  header.setTotalLength(m_reader_ptr->readUINT32LittleEndian(data_ptr,8));
//  std::cout << header.getTotalLength() << std::endl;

  header.setIdentification(m_reader_ptr->readUINT32LittleEndian(data_ptr,12));
//  std::cout << header.getIdentification() << std::endl;

  header.setFragmentOffset(m_reader_ptr->readUINT32LittleEndian(data_ptr,16));
//  std::cout << header.getFragmentOffset() << std::endl;

  return true;
}

}
}

