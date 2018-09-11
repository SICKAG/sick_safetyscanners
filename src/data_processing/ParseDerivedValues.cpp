#include <sick_microscan3_ros_driver/data_processing/ParseDerivedValues.h>

namespace sick {
namespace data_processing {

ParseDerivedValues::ParseDerivedValues()
{
  m_reader_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

datastructure::DerivedValues ParseDerivedValues::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{

  //TODO
  std::cout << "Beginn Parsing DerivedValues" << std::endl;

  //TODO sanity checks and finalize the division for the angles
  if ( data.getDataHeaderPtr()->getDerivedValuesBlockOffset() == 0 && data.getDataHeaderPtr()->getDerivedValuesBlockSize() == 0) {
    return datastructure::DerivedValues();
  }

  const BYTE* dataPtr(buffer.getBuffer().data() + data.getDataHeaderPtr()->getDerivedValuesBlockOffset());

  datastructure::DerivedValues derived_values;

  derived_values.setMultiplicationFactor(m_reader_ptr->readUINT16LittleEndian(dataPtr, 0));

  derived_values.setNumberOfBeams(m_reader_ptr->readUINT16LittleEndian(dataPtr, 2));

  derived_values.setScanTime(m_reader_ptr->readUINT16LittleEndian(dataPtr, 4));

  derived_values.setStartAngle(m_reader_ptr->readINT32LittleEndian(dataPtr, 8));

  derived_values.setAngularBeamResolution(m_reader_ptr->readINT32LittleEndian(dataPtr, 12) );

  derived_values.setInterbeamPeriod(m_reader_ptr->readUINT32LittleEndian(dataPtr, 16));

  return derived_values;
}

}
}

