#include <sick_microscan3_ros_driver/data_processing/ParseApplicationData.h>

namespace sick {
namespace data_processing {

//TODO class with both in and outputs
datastructure::ApplicationData ParseApplicationData::parseUDPSequence(datastructure::PacketBuffer buffer, datastructure::Data &data)
{
  std::cout << "Beginn Parsing ParseApplicationData" << std::endl;

  //TODO sanity checks
  if ( data.getDataHeaderPtr()->getApplicationDataBlockOffset() == 0 && data.getDataHeaderPtr()->getApplicationDataBlockSize() == 0) {
    return datastructure::ApplicationData();
  }

  const BYTE* dataPtr(buffer.getBuffer().data() + data.getDataHeaderPtr()->getApplicationDataBlockOffset());

  datastructure::ApplicationData application_data;

  datastructure::ApplicationInputs inputs;

  UINT32 word32 = ReadWriteHelper::readUINT32LE(dataPtr);

  std::vector<bool> input_sources;
  // TODO 32 dependend on word?
  for (int i = 0; i < 32; i++)
  {
     input_sources.push_back(static_cast<bool>(word32 & (0x01 << i)));
  }
  inputs.setUnsafeInputsInputSources(input_sources);

  word32 = ReadWriteHelper::readUINT32LE(dataPtr);

  std::vector<bool> input_flags;
  // TODO 32 dependend on word?
  for (int i = 0; i < 32; i++)
  {
     input_flags.push_back(static_cast<bool>(word32 & (0x01 << i)));
  }
  inputs.setUnsafeInputsFlags(input_flags);

  //Reserved 4 Byte
  UINT8 word8 = ReadWriteHelper::readUINT8LE(dataPtr);
  word8 = ReadWriteHelper::readUINT8LE(dataPtr);
  word8 = ReadWriteHelper::readUINT8LE(dataPtr);
  word8 = ReadWriteHelper::readUINT8LE(dataPtr);

  std::vector<UINT16> monitoring_cases;

  UINT16 word16;
  for (int i = 0 ; i < 20; i++) {
    monitoring_cases.push_back(ReadWriteHelper::readUINT16LE(dataPtr));
  }
  inputs.setMonitoringCase(monitoring_cases);

  word32 = ReadWriteHelper::readUINT32LE(dataPtr);

  std::vector<bool> monitoring_flags;
  // 20 for each case one
  for (int i = 0; i < 20; i++)
  {
     monitoring_flags.push_back(static_cast<bool>(word32 & (0x01 << i)));
  }
  inputs.setMonitoringCaseFlags(monitoring_flags);

  inputs.setVelocity0(ReadWriteHelper::readUINT16LE(dataPtr));
  inputs.setVelocity1(ReadWriteHelper::readUINT16LE(dataPtr));

  word8 = ReadWriteHelper::readUINT8LE(dataPtr);

  inputs.setVelocity0Valid(static_cast<bool>(word8 & (0x01 << 0)));
  inputs.setVelocity1Valid(static_cast<bool>(word8 & (0x01 << 1)));
  //reserved bits 2,3
  inputs.setVelocity0TransmittedSafely(static_cast<bool>(word8 & (0x01 << 4)));
  inputs.setVelocity1TransmittedSafely(static_cast<bool>(word8 & (0x01 << 5)));
  //reserved bits 6,7

  //reserved 1 Byte for linear velocities
  ReadWriteHelper::readUINT8LE(dataPtr);

  //reserved 2 Byte
  ReadWriteHelper::readUINT8LE(dataPtr);
  ReadWriteHelper::readUINT8LE(dataPtr);

  //reserved 10 Byte
  for (int i = 0; i < 10; i++) {
    ReadWriteHelper::readUINT8LE(dataPtr);
  }

  inputs.setSleepModeInput(ReadWriteHelper::readUINT8LE(dataPtr));

  //Reserved 1 Byte
  ReadWriteHelper::readUINT8LE(dataPtr);

  //reserved 64 Byte
  for (int i = 0; i < 64; i++) {
    ReadWriteHelper::readUINT8LE(dataPtr);
  }

  application_data.setInputs(inputs);

  //############## Outputs ############

  datastructure::ApplicationOutputs outputs;

  word32 = ReadWriteHelper::readUINT32LE(dataPtr);

  std::vector<bool> eval_out;
  for (int i = 0; i < 20 ; i++) {
    eval_out.push_back(word32 & (0x01 << i));
  }
  outputs.setEvalOut(eval_out);

  word32 = ReadWriteHelper::readUINT32LE(dataPtr);

  std::vector<bool> eval_out_is_safe;
  for (int i = 0; i < 20 ; i++) {
    eval_out_is_safe.push_back(word32 & (0x01 << i));
  }
  outputs.setEvalOutIsSafe(eval_out_is_safe);

  word32 = ReadWriteHelper::readUINT32LE(dataPtr);

  std::vector<bool> eval_out_is_valid;
  for (int i = 0; i < 20 ; i++) {
    eval_out_is_valid.push_back(word32 & (0x01 << i));
  }
  outputs.setEvalOutIsValid(eval_out_is_valid);

  std::vector<UINT16> output_monitoring_cases;

  for (int i = 0 ; i < 20; i++) {
    output_monitoring_cases.push_back(ReadWriteHelper::readUINT16LE(dataPtr));
  }
  outputs.setMonitoringCase(output_monitoring_cases);

  word32 = ReadWriteHelper::readUINT32LE(dataPtr);

  std::vector<bool> output_monitoring_flags;
  // 20 for each case one
  for (int i = 0; i < 20; i++)
  {
     output_monitoring_flags.push_back(static_cast<bool>(word32 & (0x01 << i)));
  }
  outputs.setMonitoringCaseFlags(output_monitoring_flags);

  outputs.setSleepModeOutput(ReadWriteHelper::readUINT8LE(dataPtr));

  word8 = ReadWriteHelper::readUINT8LE(dataPtr);

  outputs.setHostErrorFlagContaminationWarning(static_cast<bool>(word8 & (0x01 << 0)));
  outputs.setHostErrorFlagContaminationError(static_cast<bool>(word8 & (0x01 << 1)));
  outputs.setHostErrorFlagManipulationError(static_cast<bool>(word8 & (0x01 << 2)));
  outputs.setHostErrorFlagGlare(static_cast<bool>(word8 & (0x01 << 3)));
  outputs.setHostErrorFlagReferenceContourIntruded(static_cast<bool>(word8 & (0x01 << 4)));
  outputs.setHostErrorFlagCriticalError(static_cast<bool>(word8 & (0x01 << 5)));
  //bit 6,7 reserved

  //4Byte for error flags reserved
  ReadWriteHelper::readUINT8LE(dataPtr);
  ReadWriteHelper::readUINT8LE(dataPtr);
  ReadWriteHelper::readUINT8LE(dataPtr);
  ReadWriteHelper::readUINT8LE(dataPtr);

  //2Byte reserved
  ReadWriteHelper::readUINT8LE(dataPtr);
  ReadWriteHelper::readUINT8LE(dataPtr);

  outputs.setVelocity0(ReadWriteHelper::readUINT16LE(dataPtr));
  outputs.setVelocity1(ReadWriteHelper::readUINT16LE(dataPtr));

  word8 = ReadWriteHelper::readUINT8LE(dataPtr);

  outputs.setVelocity0Valid(static_cast<bool>(word8 & (0x01 << 0)));
  outputs.setVelocity1Valid(static_cast<bool>(word8 & (0x01 << 1)));
  //reserved bits 2,3
  outputs.setVelocity0TransmittedSafely(static_cast<bool>(word8 & (0x01 << 4)));
  outputs.setVelocity1TransmittedSafely(static_cast<bool>(word8 & (0x01 << 5)));
  //reserved bits 6,7

  //1Byte reserved for velocities
  ReadWriteHelper::readUINT8LE(dataPtr);

  //2 Byte reserved
  ReadWriteHelper::readUINT8LE(dataPtr);
  ReadWriteHelper::readUINT8LE(dataPtr);

  std::vector<INT16> resulting_velocities;
  for (int i = 0 ; i < 20; i++) {
    resulting_velocities.push_back(ReadWriteHelper::readINT16LE(dataPtr));
  }
  outputs.setResultingVelocity(resulting_velocities);

  word32 = ReadWriteHelper::readUINT32LE(dataPtr);

  std::vector<bool> resulting_velocities_flags;
  // 20 for each case one
  for (int i = 0; i < 20; i++)
  {
     resulting_velocities_flags.push_back(static_cast<bool>(word32 & (0x01 << i)));
  }
  outputs.setResultingVelocityIsValid(resulting_velocities_flags);


  //reserved 7 Byte
  for (int i = 0; i < 7; i++) {
    ReadWriteHelper::readUINT8LE(dataPtr);
  }

  word8 = ReadWriteHelper::readUINT8LE(dataPtr);

  outputs.setFlagsSleepModeOutputIsValid(static_cast<bool>(word8 & (0x01 << 0)));
  outputs.setFlagsHostErrorFlagsAreValid(static_cast<bool>(word8 & (0x01 << 1)));
  //bit 2-7 are reserved

  application_data.setOutputs(outputs);

  return application_data;
}

}
}

