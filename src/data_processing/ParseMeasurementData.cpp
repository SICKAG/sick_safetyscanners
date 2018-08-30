#include <sick_microscan3_ros_driver/data_processing/ParseMeasurementData.h>

namespace sick {
namespace data_processing {

datastructure::MeasurementData ParseMeasurementData::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{
  std::cout << "Beginn Parsing Header" << std::endl;

  const BYTE* dataPtr(buffer.getBuffer().data() + data.getDataHeaderPtr()->getMeasurementDataBlockOffset());

  datastructure::MeasurementData measurement_data;

  measurement_data.setNumberOfBeams(ReadWriteHelper::readUINT32LE(dataPtr));
  std::cout << "NumberOfBeams: " << measurement_data.getNumberOfBeams() << std::endl;

  float angle = data.getDerivedValuesPtr()->getStartAngle();
  std::cout << angle << std::endl;
  float angle_delta = data.getDerivedValuesPtr()->getAngularBeamResolution();

  for (int i = 0; i < measurement_data.getNumberOfBeams(); i++)
  {
    INT16 distance = ReadWriteHelper::readUINT16LE(dataPtr);

    UINT8 reflectivity = ReadWriteHelper::readUINT8LE(dataPtr);

    UINT8 status = ReadWriteHelper::readUINT8LE(dataPtr);


    //TODO
    bool valid = status & (0x01 << 0);

    bool infinite = status & (0x01 << 1);

    bool glare = status & (0x01 << 2);

    bool reflector = status & (0x01 << 3);

    bool contamination = status & (0x01 << 4);

    bool contamination_warning = status & (0x01 << 5);

    measurement_data.addScanPoint(sick::datastructure::ScanPoint(angle, distance, reflectivity, valid, infinite, glare, reflector, contamination, contamination_warning));

    angle += angle_delta;

  }

  std::cout  << "measurement data size: " << measurement_data.getScanPoints().size() << std::endl;

//  std::cout  << "measurement data at 10: " << measurement_data.getScanPoints().at(714).getDistance() << std::endl;
//  std::cout  << "measurement data at 10: " << measurement_data.getScanPoints().at(714).getAngle() << std::endl;
//  std::cout  << "measurement data at 10: " << measurement_data.getScanPoints().at(714).getValidBit() << std::endl;

  return measurement_data;
}

}
}

