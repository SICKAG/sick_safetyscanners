#pragma once

#include <sick_microscan3_ros_driver/datastructure/PaketBuffer.h>
#include <sick_microscan3_ros_driver/data_processing/ParseDatagramHeader.h>

namespace sick {
namespace data_processing {

class UDPPaketMerger
{
public:
  UDPPaketMerger();

  bool isComplete();

  bool addUDPPaket(sick::datastructure::PaketBuffer buffer);
private:
  bool m_is_complete;
};

}
}


