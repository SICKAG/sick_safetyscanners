#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/MeasurementData.h>


namespace sick {
namespace data_processing {

class ParseMeasurementData : public AbstractParseUDPSequence
{
public:
  ParseMeasurementData();

  datastructure::MeasurementData parseUDPSequence(sick::datastructure::PacketBuffer buffer, datastructure::Data &header);

private:
  float m_angle;
  float m_angle_delta;
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  bool setDataInMeasurementData(const BYTE *data_ptr, datastructure::MeasurementData &measurement_data);
  bool setNumberOfBeamsInMeasurementData(const BYTE *data_ptr, datastructure::MeasurementData &measurement_data);
  bool setStartAngleAndDelta(datastructure::Data &data);
  bool setScanPointsInMeasurementData(const BYTE *data_ptr, datastructure::MeasurementData &measurement_data);
  bool addScanPointToMeasurementData(UINT16 offset, const BYTE *data_ptr, datastructure::MeasurementData &measurement_data);
};

}
}


