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

  boost::shared_ptr<sick::data_processing::ParseDataHeader> m_data_header_parser_ptr;
  boost::shared_ptr<sick::data_processing::ParseDerivedValues> m_derived_values_parser_ptr;
  boost::shared_ptr<sick::data_processing::ParseMeasurementData> m_measurement_data_parser_ptr;
  boost::shared_ptr<sick::data_processing::ParseGeneralSystemState> m_general_system_state_parser_ptr;
  boost::shared_ptr<sick::data_processing::ParseIntrusionData> m_intrusion_data_parser_ptr;
  boost::shared_ptr<sick::data_processing::ParseApplicationData> m_application_data_parser_ptr;


  bool setDataBlocksInData(datastructure::PacketBuffer &buffer, datastructure::Data &data);
  bool setDataHeaderInData(datastructure::PacketBuffer &buffer, datastructure::Data &data);
  bool setDerivedValuesInData(datastructure::PacketBuffer &buffer, datastructure::Data &data);
  bool setMeasurementDataInData(datastructure::PacketBuffer &buffer, datastructure::Data &data);
  bool setGeneralSystemStateInData(datastructure::PacketBuffer &buffer, datastructure::Data &data);
  bool setIntrusionDataInData(datastructure::PacketBuffer &buffer, datastructure::Data &data);
  bool setApplicationDataInData(datastructure::PacketBuffer &buffer, datastructure::Data &data);
};

}
}


