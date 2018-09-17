#include <sick_microscan3_ros_driver/data_processing/ParseMeasurementData.h>

namespace sick {
namespace data_processing {

ParseMeasurementData::ParseMeasurementData()
{
  m_reader_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

datastructure::MeasurementData ParseMeasurementData::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{
  std::cout << "Beginn Parsing Measrurement Data" << std::endl;

  datastructure::MeasurementData measurement_data;
  if (!checkIfPreconditionsAreMet(data)) {
    measurement_data.setIsEmpty(true);
    return measurement_data;
  }
  std::cout << "Beginn Parsing Measurement Data Conditions Met" << std::endl;


  const BYTE* data_ptr(buffer.getBuffer().data() + data.getDataHeaderPtr()->getMeasurementDataBlockOffset());

  setStartAngleAndDelta(data);
  setDataInMeasurementData(data_ptr, measurement_data);
  return measurement_data;
}

bool ParseMeasurementData::checkIfPreconditionsAreMet(datastructure::Data &data)
{
  if (!checkIfMeasurementDataIsPublished(data))
  {
    return false;
  }
  if (!checkIfDataContainsNeededParsedBlocks(data))
  {
    return false;
  }
  return true;
}

bool ParseMeasurementData::checkIfMeasurementDataIsPublished(datastructure::Data &data)
{
  if ( data.getDataHeaderPtr()->getMeasurementDataBlockOffset() == 0 && data.getDataHeaderPtr()->getMeasurementDataBlockSize() == 0)
  {
    return false;
  }
  return true;
}

bool ParseMeasurementData::checkIfDataContainsNeededParsedBlocks(datastructure::Data &data)
{
  if (data.getDataHeaderPtr()->isEmpty()){
    return false;
  }
  if (data.getDerivedValuesPtr()->isEmpty()){
    return false;
  }
  return true;
}


bool ParseMeasurementData::setDataInMeasurementData(const BYTE* data_ptr, datastructure::MeasurementData &measurement_data)
{
  setNumberOfBeamsInMeasurementData(data_ptr, measurement_data);
  std::cout << "NumberOfBeams: " << measurement_data.getNumberOfBeams() << std::endl;
  setScanPointsInMeasurementData(data_ptr, measurement_data);
  std::cout  << "measurement data size: " << measurement_data.getScanPointsVector().size() << std::endl;

}

bool ParseMeasurementData::setNumberOfBeamsInMeasurementData(const BYTE* data_ptr, datastructure::MeasurementData &measurement_data)
{
  measurement_data.setNumberOfBeams(m_reader_ptr->readUINT32LittleEndian(data_ptr,0));
}

bool ParseMeasurementData::setStartAngleAndDelta(datastructure::Data &data)
{
  m_angle = data.getDerivedValuesPtr()->getStartAngle();
  std::cout << m_angle << std::endl;
  m_angle_delta = data.getDerivedValuesPtr()->getAngularBeamResolution();
}

bool ParseMeasurementData::setScanPointsInMeasurementData(const BYTE* data_ptr, datastructure::MeasurementData &measurement_data)
{
  for (int i = 0; i < measurement_data.getNumberOfBeams(); i++)
  {
    addScanPointToMeasurementData(i, data_ptr, measurement_data);
    m_angle += m_angle_delta;
  }
}

bool ParseMeasurementData::addScanPointToMeasurementData(UINT16 offset, const BYTE* data_ptr, datastructure::MeasurementData &measurement_data)
{
  INT16 distance = m_reader_ptr->readUINT16LittleEndian(data_ptr, (4 + offset * 4));
  UINT8 reflectivity = m_reader_ptr->readUINT8LittleEndian(data_ptr, (6 + offset *4) );
  UINT8 status = m_reader_ptr->readUINT8LittleEndian(data_ptr, (7 + offset * 4));
  bool valid = status & (0x01 << 0);
  bool infinite = status & (0x01 << 1);
  bool glare = status & (0x01 << 2);
  bool reflector = status & (0x01 << 3);
  bool contamination = status & (0x01 << 4);
  bool contamination_warning = status & (0x01 << 5);
  measurement_data.addScanPoint(sick::datastructure::ScanPoint(m_angle, distance, reflectivity, valid, infinite, glare, reflector, contamination, contamination_warning));
}

}
}

