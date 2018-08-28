#include <sick_microscan3_ros_driver/data_processing/UDPPaketMerger.h>

namespace sick {
namespace data_processing {

UDPPaketMerger::UDPPaketMerger()
  : m_is_complete(false)
{

}


bool UDPPaketMerger::isComplete()
{
   return m_is_complete;
}

bool UDPPaketMerger::addUDPPaket(sick::datastructure::PaketBuffer buffer)
{

  sick::datastructure::DatagramHeader datagram_header;
  // parse Custom Header
  sick::data_processing::ParseDatagramHeader::parseUDPSequence(buffer, datagram_header);

  //add to map

  //check if complete

  return isComplete();
}

}
}
