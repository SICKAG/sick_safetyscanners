#include <sick_microscan3_ros_driver/data_processing/ParseData.h>

namespace sick {
namespace data_processing {

ParseData::ParseData()
{
  m_reader_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

bool ParseData::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{
  std::cout << "Beginn Parsing Data Header" << std::endl;

  const BYTE* dataPtr(buffer.getBuffer().data());

  sick::data_processing::ParseDataHeader data_header_parser;
  sick::datastructure::DataHeader data_header = data_header_parser.parseUDPSequence(buffer, data);
  data.setDataHeaderPtr(boost::make_shared<sick::datastructure::DataHeader>(data_header));

  std::cout << "DATA: scanNumber:  " << data.getDataHeaderPtr()->getScanNumber() << std::endl;

  sick::data_processing::ParseDerivedValues derived_values_parser;
  sick::datastructure::DerivedValues derived_values = derived_values_parser.parseUDPSequence(buffer,data);
  data.setDerivedValuesPtr(boost::make_shared<sick::datastructure::DerivedValues>(derived_values));

  sick::data_processing::ParseMeasurementData measurement_data_parser;
  sick::datastructure::MeasurementData measurement_data = measurement_data_parser.parseUDPSequence(buffer, data);
  data.setMeasurementDataPtr(boost::make_shared<sick::datastructure::MeasurementData>(measurement_data));

  sick::data_processing::ParseGeneralSystemState general_system_state_parser;
  sick::datastructure::GeneralSystemState general_system_state = general_system_state_parser.parseUDPSequence(buffer,data);
  data.setGeneralSystemStatePtr(boost::make_shared<sick::datastructure::GeneralSystemState>(general_system_state));

  sick::data_processing::ParseIntrusionData intrusion_data_parser;
  sick::datastructure::IntrusionData intrusion_data = intrusion_data_parser.parseUDPSequence(buffer,data);
  data.setIntrusionDataPtr(boost::make_shared<sick::datastructure::IntrusionData>(intrusion_data));

  sick::data_processing::ParseApplicationData application_data_parser;
  sick::datastructure::ApplicationData application_data = application_data_parser.parseUDPSequence(buffer,data);
  data.setApplicationDataPtr(boost::make_shared<sick::datastructure::ApplicationData>(application_data));



  return true;
}

}
}

