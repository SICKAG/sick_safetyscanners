#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>


namespace sick {
namespace data_processing {

class ParseGeneralSystemState : public AbstractParseUDPSequence
{
public:
  ParseGeneralSystemState();

  datastructure::GeneralSystemState parseUDPSequence(sick::datastructure::PacketBuffer buffer, datastructure::Data &data);

private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  bool setDataInGeneralSystemState(const BYTE *data_ptr, datastructure::GeneralSystemState &general_System_state);
  bool setStatusBitsInGeneralSystemState(const BYTE *data_ptr, datastructure::GeneralSystemState &general_System_state);
  bool setSafeCutOffPathInGeneralSystemState(const BYTE *data_ptr, datastructure::GeneralSystemState &general_System_state);
  bool setNonSafeCutOffPathInGeneralSystemState(const BYTE *data_ptr, datastructure::GeneralSystemState &general_System_state);
  bool setResetRequiredCutOffPathInGeneralSystemState(const BYTE *data_ptr, datastructure::GeneralSystemState &general_System_state);
  bool setCurrentMonitoringCasesInGeneralSystemState(const BYTE *data_ptr, datastructure::GeneralSystemState &general_System_state);
  bool setErrorsInGeneralSystemState(const BYTE *data_ptr, datastructure::GeneralSystemState &general_System_state);
  bool checkIfPreconditionsAreMet(datastructure::Data &data);
  bool checkIfGeneralSystemStateIsPublished(datastructure::Data &data);
  bool checkIfDataContainsNeededParsedBlocks(datastructure::Data &data);
};

}
}


