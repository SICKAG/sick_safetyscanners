#pragma once

#include <sick_microscan3_ros_driver/datastructure/PacketBuffer.h>
#include <sick_microscan3_ros_driver/datastructure/Data.h>
#include <sick_microscan3_ros_driver/data_processing/ReadWriteHelper.h>
#include <boost/make_shared.hpp>


namespace sick {
namespace data_processing {

class AbstractParseUDPSequence
{
public:
  AbstractParseUDPSequence()
  {
//    m_readerPtr = boost::make_shared<sick::data_processing::ReadWriteHelper>();
  }
  //TODO add Template second argument or something similar
  static bool parseUDPSequence(sick::datastructure::PacketBuffer buffer);

  boost::shared_ptr<sick::data_processing::ReadWriteHelper> readerPtr() const;
  void setReaderPtr(const boost::shared_ptr<sick::data_processing::ReadWriteHelper> &readerPtr);

private:
//  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_readerPtr;

};





}
}


