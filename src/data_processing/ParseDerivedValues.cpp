#include <sick_microscan3_ros_driver/data_processing/ParseDerivedValues.h>

namespace sick {
namespace data_processing {

datastructure::DerivedValues ParseDerivedValues::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{
  std::cout << "Beginn Parsing DerivedValues" << std::endl;

  //TODO sanity checks and finalize the division for the angles
  if ( data.getDataHeaderPtr()->getDerivedValuesBlockOffset() == 0 && data.getDataHeaderPtr()->getDerivedValuesBlockSize() == 0) {
    return datastructure::DerivedValues();
  }

  const BYTE* dataPtr(buffer.getBuffer().data() + data.getDataHeaderPtr()->getDerivedValuesBlockOffset());

  datastructure::DerivedValues derived_values;

  derived_values.setMultiplicationFactor(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "MultiplicationFactor: " << derived_values.getMultiplicationFactor() << std::endl;

  derived_values.setNumberOfBeams(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "NumberOfBeams: " << derived_values.getNumberOfBeams() << std::endl;

  derived_values.setScanTime(ReadWriteHelper::readUINT16LE(dataPtr));
  std::cout << "ScanTime: " << derived_values.getScanTime() << std::endl;

  //2 Bytes Reserved
  ReadWriteHelper::readUINT8LE(dataPtr);
  ReadWriteHelper::readUINT8LE(dataPtr);


  derived_values.setStartAngle(ReadWriteHelper::readINT32LE(dataPtr));
  std::cout << "StartAngle: " << derived_values.getStartAngle() << std::endl;

  derived_values.setAngularBeamResolution(ReadWriteHelper::readINT32LE(dataPtr) );
  std::cout << "AngularBeamResolution: " << derived_values.getAngularBeamResolution() << std::endl;

  derived_values.setInterbeamPeriod(ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << "InterbeamPeriod: " << derived_values.getInterbeamPeriod() << std::endl;

  // 4 bytes Reserved -- skipped since no data afterwars for derived values


  return derived_values;
}

}
}

