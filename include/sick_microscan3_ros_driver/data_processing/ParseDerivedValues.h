#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>


namespace sick {
namespace data_processing {

class ParseDerivedValues : public AbstractParseUDPSequence
{
public:
  ParseDerivedValues();

  datastructure::DerivedValues parseUDPSequence(sick::datastructure::PacketBuffer buffer, datastructure::Data &data);

private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  bool setDataInDerivedValues(const BYTE *data_ptr, datastructure::DerivedValues &derived_values);
  bool setMultiplicationFactorInDerivedValues(const BYTE *data_ptr, datastructure::DerivedValues &derived_values);
  bool setNumberOfBeamsInDerivedValues(const BYTE *data_ptr, datastructure::DerivedValues &derived_values);
  bool setScanTimeInDerivedValues(const BYTE *data_ptr, datastructure::DerivedValues &derived_values);
  bool setStartAngleInDerivedValues(const BYTE *data_ptr, datastructure::DerivedValues &derived_values);
  bool setAngularBeamResolutionInDerivedValues(const BYTE *data_ptr, datastructure::DerivedValues &derived_values);
  bool setInterbeamPeriodInDerivedValues(const BYTE *data_ptr, datastructure::DerivedValues &derived_values);
  bool checkIfPreconditionsAreMet(datastructure::Data &data);
  bool checkIfDerivedValuesIsPublished(datastructure::Data &data);
  bool checkIfDataContainsNeededParsedBlocks(datastructure::Data &data);
};

}
}


