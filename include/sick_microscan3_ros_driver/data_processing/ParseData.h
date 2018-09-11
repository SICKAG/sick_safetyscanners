#pragma once

#include <boost/make_shared.hpp>

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/Data.h>
#include <sick_microscan3_ros_driver/data_processing/ParseDataHeader.h>
#include <sick_microscan3_ros_driver/data_processing/ParseDerivedValues.h>
#include <sick_microscan3_ros_driver/data_processing/ParseMeasurementData.h>
#include <sick_microscan3_ros_driver/data_processing/ParseGeneralSystemState.h>
#include <sick_microscan3_ros_driver/data_processing/ParseIntrusionData.h>
#include <sick_microscan3_ros_driver/data_processing/ParseApplicationData.h>

namespace sick {
namespace data_processing {

class ParseData : public AbstractParseUDPSequence
{
public:
  ParseData();

  bool parseUDPSequence(sick::datastructure::PacketBuffer buffer, sick::datastructure::Data& data);

private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
};

}
}


