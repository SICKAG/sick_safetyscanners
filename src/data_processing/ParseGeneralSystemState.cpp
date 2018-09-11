#include <sick_microscan3_ros_driver/data_processing/ParseGeneralSystemState.h>

namespace sick {
namespace data_processing {

ParseGeneralSystemState::ParseGeneralSystemState()
{
  m_reader_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();

}

datastructure::GeneralSystemState ParseGeneralSystemState::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{
  std::cout << "Beginn Parsing General System State" << std::endl;

  //TODO sanity checks and finalize the division for the angles
  if ( data.getDataHeaderPtr()->getGeneralSystemStateBlockOffset() == 0 && data.getDataHeaderPtr()->getGeneralSystemStateBlockSize() == 0) {
    return datastructure::GeneralSystemState();
  }

  const BYTE* data_ptr(buffer.getBuffer().data() + data.getDataHeaderPtr()->getGeneralSystemStateBlockOffset());

  datastructure::GeneralSystemState general_system_state;

  UINT8 byte = m_reader_ptr->readUINT8LittleEndian(data_ptr,0);

  general_system_state.setRunModeActive(static_cast<bool>(byte & (0x01 << 0)));
  general_system_state.setStandbyModeActive(static_cast<bool>(byte & (0x01 << 1)));
  general_system_state.setContaminationWarning(static_cast<bool>(byte & (0x01 << 2)));
  general_system_state.setContaminationError(static_cast<bool>(byte & (0x01 << 3)));
  general_system_state.setReferenceContourStatus(static_cast<bool>(byte & (0x01 << 4)));
  general_system_state.setManipulationStatus(static_cast<bool>(byte & (0x01 << 5)));
  // bit 6 and 7 reserved

  std::vector<bool> safe_cut_off_path;

  for (int i = 0; i < 3; i++) {
    byte = m_reader_ptr->readUINT8LittleEndian(data_ptr, 1 + i);

    for (int j = 0; j < 8; j++) {
      //as long as there are only 20 instead of 24 cut off paths
      if(i ==2 && j > 3) {
        break;
      }
      safe_cut_off_path.push_back(static_cast<bool>(byte & (0x01 << j)));
    }
  }
  general_system_state.setSafeCutOffPathvector(safe_cut_off_path);
  std::cout << "cutoffsize" << safe_cut_off_path.size() << std::endl;

  std::vector<bool> non_safe_cut_off_path;

  for (int i = 0; i < 3; i++) {
    byte = m_reader_ptr->readUINT8LittleEndian(data_ptr, 4 + i);

    for (int j = 0; j < 8; j++) {
      //as long as there are only 20 instead of 24 cut off paths
      if(i ==2 && j > 3) {
        break;
      }
      non_safe_cut_off_path.push_back(static_cast<bool>(byte & (0x01 << j)));
    }
  }
  general_system_state.setNonSafeCutOffPathVector(non_safe_cut_off_path);

  std::vector<bool> reset_required_cutoff_path;

  for (int i = 0; i < 3; i++) {
    byte = m_reader_ptr->readUINT8LittleEndian(data_ptr, 7 + i);

    for (int j = 0; j < 8; j++) {
      //as long as there are only 20 instead of 24 cut off paths
      if(i ==2 && j > 3) {
        break;
      }
      reset_required_cutoff_path.push_back(static_cast<bool>(byte & (0x01 << j)));
    }
  }
  general_system_state.setResetRequiredCutOffPathVector(reset_required_cutoff_path);

  general_system_state.setCurrentMonitoringCaseNoTable_1(m_reader_ptr->readUINT8LittleEndian(data_ptr,10));
  general_system_state.setCurrentMonitoringCaseNoTable_2(m_reader_ptr->readUINT8LittleEndian(data_ptr,11));
  general_system_state.setCurrentMonitoringCaseNoTable_3(m_reader_ptr->readUINT8LittleEndian(data_ptr,12));
  general_system_state.setCurrentMonitoringCaseNoTable_4(m_reader_ptr->readUINT8LittleEndian(data_ptr,13));

  //reserved byte
//  m_reader_ptr->readUINT8LittleEndian(data_ptr);

  byte = m_reader_ptr->readUINT8LittleEndian(data_ptr,15);
  general_system_state.setApplicationError(static_cast<bool>(byte & (0x01 << 0)));
  general_system_state.setDeviceError(static_cast<bool>(byte & (0x01 << 1)));
  //bit 2-7 are reserved

  return general_system_state;
}

}
}

