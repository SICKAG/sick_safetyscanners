#include <sick_microscan3_ros_driver/data_processing/ParseApplicationData.h>

namespace sick {
namespace data_processing {

ParseApplicationData::ParseApplicationData()
{
  m_reader_ptr = boost::make_shared<sick::data_processing::ReadWriteHelper>();
}

//TODO class with both in and outputs
datastructure::ApplicationData ParseApplicationData::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{
  std::cout << "Beginn Parsing ParseApplicationData" << std::endl;

  //TODO sanity checks
  if ( data.getDataHeaderPtr()->getApplicationDataBlockOffset() == 0 && data.getDataHeaderPtr()->getApplicationDataBlockSize() == 0) {
    return datastructure::ApplicationData();
  }

  const BYTE* data_ptr(buffer.getBuffer().data() + data.getDataHeaderPtr()->getApplicationDataBlockOffset());

  datastructure::ApplicationData application_data;

  datastructure::ApplicationInputs inputs;

  UINT32 word32 = m_reader_ptr->readUINT32LittleEndian(data_ptr, 0);

  std::vector<bool> input_sources;
  // TODO 32 dependend on word?
  for (int i = 0; i < 32; i++)
  {
     input_sources.push_back(static_cast<bool>(word32 & (0x01 << i)));
  }
  inputs.setUnsafeInputsInputSourcesVector(input_sources);

  word32 = m_reader_ptr->readUINT32LittleEndian(data_ptr,4);

  std::vector<bool> input_flags;
  // TODO 32 dependend on word?
  for (int i = 0; i < 32; i++)
  {
     input_flags.push_back(static_cast<bool>(word32 & (0x01 << i)));
  }
  inputs.setUnsafeInputsFlagsVector(input_flags);

//  //Reserved 4 Byte
//  UINT8 word8 = m_reader_ptr->readUINT8LittleEndian(data_ptr,8);
//  word8 = m_reader_ptr->readUINT8LittleEndian(data_ptr,9);
//  word8 = m_reader_ptr->readUINT8LittleEndian(data_ptr,10);
//  word8 = m_reader_ptr->readUINT8LittleEndian(data_ptr,11);

  std::vector<UINT16> monitoring_cases;

  UINT16 word16;
  for (int i = 0 ; i < 20; i++) {
    monitoring_cases.push_back(m_reader_ptr->readUINT16LittleEndian(data_ptr, 12 + i * 2));
  }
  inputs.setMonitoringCaseVector(monitoring_cases);

  word32 = m_reader_ptr->readUINT32LittleEndian(data_ptr, 52);

  std::vector<bool> monitoring_flags;
  // 20 for each case one
  for (int i = 0; i < 20; i++)
  {
     monitoring_flags.push_back(static_cast<bool>(word32 & (0x01 << i)));
  }
  inputs.setMonitoringCaseFlagsVector(monitoring_flags);

  inputs.setVelocity0(m_reader_ptr->readUINT16LittleEndian(data_ptr,56));
  inputs.setVelocity1(m_reader_ptr->readUINT16LittleEndian(data_ptr,58));

  UINT8 word8 = m_reader_ptr->readUINT8LittleEndian(data_ptr,60);

  inputs.setVelocity0Valid(static_cast<bool>(word8 & (0x01 << 0)));
  inputs.setVelocity1Valid(static_cast<bool>(word8 & (0x01 << 1)));
  //reserved bits 2,3
  inputs.setVelocity0TransmittedSafely(static_cast<bool>(word8 & (0x01 << 4)));
  inputs.setVelocity1TransmittedSafely(static_cast<bool>(word8 & (0x01 << 5)));
  //reserved bits 6,7

  //reserved 1 Byte for linear velocities
//  m_reader_ptr->readUINT8LittleEndian(data_ptr,61);

//  //reserved 2 Byte
//  m_reader_ptr->readUINT8LittleEndian(data_ptr,62);
//  m_reader_ptr->readUINT8LittleEndian(data_ptr,63);

  //reserved 10 Byte
//  for (int i = 0; i < 10; i++) {
//    m_reader_ptr->readUINT8LittleEndian(data_ptr);
//  }

  inputs.setSleepModeInput(m_reader_ptr->readUINT8LittleEndian(data_ptr,74));

//  //Reserved 1 Byte
//  m_reader_ptr->readUINT8LittleEndian(data_ptr);

//  //reserved 64 Byte
//  for (int i = 0; i < 64; i++) {
//    m_reader_ptr->readUINT8LittleEndian(data_ptr);
//  }

  application_data.setInputs(inputs);

  //############## Outputs ############

  datastructure::ApplicationOutputs outputs;

  word32 = m_reader_ptr->readUINT32LittleEndian(data_ptr, 140);

  std::vector<bool> eval_out;
  for (int i = 0; i < 20 ; i++) {
    eval_out.push_back(word32 & (0x01 << i));
  }
  outputs.setEvalOutVector(eval_out);

  word32 = m_reader_ptr->readUINT32LittleEndian(data_ptr,144);

  std::vector<bool> eval_out_is_safe;
  for (int i = 0; i < 20 ; i++) {
    eval_out_is_safe.push_back(word32 & (0x01 << i));
  }
  outputs.setEvalOutIsSafeVector(eval_out_is_safe);

  word32 = m_reader_ptr->readUINT32LittleEndian(data_ptr,148);

  std::vector<bool> eval_out_is_valid;
  for (int i = 0; i < 20 ; i++) {
    eval_out_is_valid.push_back(word32 & (0x01 << i));
  }
  outputs.setEvalOutIsValidVector(eval_out_is_valid);

  std::vector<UINT16> output_monitoring_cases;

  for (int i = 0 ; i < 20; i++) {
    output_monitoring_cases.push_back(m_reader_ptr->readUINT16LittleEndian(data_ptr, 152 + i*2));
  }
  outputs.setMonitoringCaseVector(output_monitoring_cases);

  word32 = m_reader_ptr->readUINT32LittleEndian(data_ptr,192);

  std::vector<bool> output_monitoring_flags;
  // 20 for each case one
  for (int i = 0; i < 20; i++)
  {
     output_monitoring_flags.push_back(static_cast<bool>(word32 & (0x01 << i)));
  }
  outputs.setMonitoringCaseFlagsVector(output_monitoring_flags);

  outputs.setSleepModeOutput(m_reader_ptr->readUINT8LittleEndian(data_ptr,193));

  word8 = m_reader_ptr->readUINT8LittleEndian(data_ptr,194);

  outputs.setHostErrorFlagContaminationWarning(static_cast<bool>(word8 & (0x01 << 0)));
  outputs.setHostErrorFlagContaminationError(static_cast<bool>(word8 & (0x01 << 1)));
  outputs.setHostErrorFlagManipulationError(static_cast<bool>(word8 & (0x01 << 2)));
  outputs.setHostErrorFlagGlare(static_cast<bool>(word8 & (0x01 << 3)));
  outputs.setHostErrorFlagReferenceContourIntruded(static_cast<bool>(word8 & (0x01 << 4)));
  outputs.setHostErrorFlagCriticalError(static_cast<bool>(word8 & (0x01 << 5)));
  //bit 6,7 reserved

  //4Byte for error flags reserved
//  m_reader_ptr->readUINT8LittleEndian(data_ptr);
//  m_reader_ptr->readUINT8LittleEndian(data_ptr);
//  m_reader_ptr->readUINT8LittleEndian(data_ptr);
//  m_reader_ptr->readUINT8LittleEndian(data_ptr);

  //2Byte reserved
//  m_reader_ptr->readUINT8LittleEndian(data_ptr);
//  m_reader_ptr->readUINT8LittleEndian(data_ptr);

  outputs.setVelocity0(m_reader_ptr->readUINT16LittleEndian(data_ptr,200));
  outputs.setVelocity1(m_reader_ptr->readUINT16LittleEndian(data_ptr,202));

  word8 = m_reader_ptr->readUINT8LittleEndian(data_ptr,204);

  outputs.setVelocity0Valid(static_cast<bool>(word8 & (0x01 << 0)));
  outputs.setVelocity1Valid(static_cast<bool>(word8 & (0x01 << 1)));
  //reserved bits 2,3
  outputs.setVelocity0TransmittedSafely(static_cast<bool>(word8 & (0x01 << 4)));
  outputs.setVelocity1TransmittedSafely(static_cast<bool>(word8 & (0x01 << 5)));
  //reserved bits 6,7

  //1Byte reserved for velocities
//  m_reader_ptr->readUINT8LittleEndian(data_ptr);

//  //2 Byte reserved
//  m_reader_ptr->readUINT8LittleEndian(data_ptr);
//  m_reader_ptr->readUINT8LittleEndian(data_ptr);

  std::vector<INT16> resulting_velocities;
  for (int i = 0 ; i < 20; i++) {
    resulting_velocities.push_back(m_reader_ptr->readINT16LittleEndian(data_ptr,208 + i * 2));
  }
  outputs.setResultingVelocityVector(resulting_velocities);

  word32 = m_reader_ptr->readUINT32LittleEndian(data_ptr, 248);

  std::vector<bool> resulting_velocities_flags;
  // 20 for each case one
  for (int i = 0; i < 20; i++)
  {
     resulting_velocities_flags.push_back(static_cast<bool>(word32 & (0x01 << i)));
  }
  outputs.setResultingVelocityIsValidVector(resulting_velocities_flags);


  //reserved 7 Byte
//  for (int i = 0; i < 7; i++) {
//    m_reader_ptr->readUINT8LittleEndian(data_ptr);
//  }

  word8 = m_reader_ptr->readUINT8LittleEndian(data_ptr,259);

  outputs.setFlagsSleepModeOutputIsValid(static_cast<bool>(word8 & (0x01 << 0)));
  outputs.setFlagsHostErrorFlagsAreValid(static_cast<bool>(word8 & (0x01 << 1)));
  //bit 2-7 are reserved

  application_data.setOutputs(outputs);

  return application_data;
}



}
}

