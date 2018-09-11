#include <sick_microscan3_ros_driver/data_processing/ParseDerivedValues.h>

namespace sick {
namespace data_processing {

ParseDerivedValues::ParseDerivedValues()
{
  m_reader_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

datastructure::DerivedValues ParseDerivedValues::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{
  std::cout << "Beginn Parsing DerivedValues" << std::endl;

  //TODO sanity checks and finalize the division for the angles
  if ( data.getDataHeaderPtr()->getDerivedValuesBlockOffset() == 0 && data.getDataHeaderPtr()->getDerivedValuesBlockSize() == 0) {
    return datastructure::DerivedValues();
  }

  const BYTE* dataPtr(buffer.getBuffer().data() + data.getDataHeaderPtr()->getDerivedValuesBlockOffset());

  datastructure::DerivedValues derived_values;

  derived_values.setMultiplicationFactor(m_reader_ptr->readUINT16LittleEndian(dataPtr, 0));
//  std::cout << "MultiplicationFactor: " << derived_values.getMultiplicationFactor() << std::endl;

  derived_values.setNumberOfBeams(m_reader_ptr->readUINT16LittleEndian(dataPtr, 2));
//  std::cout << "NumberOfBeams: " << derived_values.getNumberOfBeams() << std::endl;

  derived_values.setScanTime(m_reader_ptr->readUINT16LittleEndian(dataPtr, 4));
//  std::cout << "ScanTime: " << derived_values.getScanTime() << std::endl;

  //2Bytes reserved
//  derived_values.setStartAngle(m_reader_ptr->readINT32LE(dataPtr));

  derived_values.setStartAngle(m_reader_ptr->readINT32LittleEndian(dataPtr, 8));
//  std::cout << "StartAngle: " << derived_values.getStartAngle() << std::endl;

//  derived_values.setAngularBeamResolution(m_reader_ptr->readINT32LE(dataPtr) );

  derived_values.setAngularBeamResolution(m_reader_ptr->readINT32LittleEndian(dataPtr, 12) );
//  std::cout << "AngularBeamResolution: " << derived_values.getAngularBeamResolution() << std::endl;

  derived_values.setInterbeamPeriod(m_reader_ptr->readUINT32LittleEndian(dataPtr, 16));
//  std::cout << "InterbeamPeriod: " << derived_values.getInterbeamPeriod() << std::endl;

  // 4 bytes Reserved -- skipped since no data afterwars for derived values


  return derived_values;
}

}
}

