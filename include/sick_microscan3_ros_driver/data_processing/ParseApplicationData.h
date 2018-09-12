#pragma once

#include <sick_microscan3_ros_driver/data_processing/AbstractParseUDPSequence.h>
#include <sick_microscan3_ros_driver/datastructure/DerivedValues.h>


namespace sick {
namespace data_processing {

class ParseApplicationData : public AbstractParseUDPSequence
{
public:
  ParseApplicationData();
  datastructure::ApplicationData parseUDPSequence(sick::datastructure::PacketBuffer buffer, datastructure::Data &data);

private:
  boost::shared_ptr<sick::data_processing::ReadWriteHelper> m_reader_ptr;
  bool setDataInApplicationData(const BYTE *data_ptr, datastructure::ApplicationData &application_data);
  bool setApplicationInputsInApplicationData(const BYTE *data_ptr, datastructure::ApplicationData &application_data);
  bool setApplicationOutputsInApplicationData(const BYTE *data_ptr, datastructure::ApplicationData &application_data);
  bool setDataInApplicationInputs(const BYTE *data_ptr, datastructure::ApplicationInputs &inputs);
  bool setDataInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setUnsafeInputsInApplicationInputs(const BYTE *data_ptr, datastructure::ApplicationInputs &inputs);
  bool setUnsafeInputsSourcesInApplicationInputs(const BYTE *data_ptr, datastructure::ApplicationInputs &inputs);
  bool setUnsafeInputsFlagsInApplicationInputs(const BYTE *data_ptr, datastructure::ApplicationInputs &inputs);
  bool setMonitoringCaseInputsInApplicationInputs(const BYTE *data_ptr, datastructure::ApplicationInputs &inputs);
  bool setMonitoringCaseNumbersInApplicationInputs(const BYTE *data_ptr, datastructure::ApplicationInputs &inputs);
  bool setMonitoringCaseFlagsInApplicationInputs(const BYTE *data_ptr, datastructure::ApplicationInputs &inputs);
  bool setLinearVelocityInputsInApplicationInputs(const BYTE *data_ptr, datastructure::ApplicationInputs &inputs);
  bool setLinearVelocity0InApplicationInputs(const BYTE *data_ptr, datastructure::ApplicationInputs &inputs);
  bool setLinearVelocity1InApplicationInputs(const BYTE *data_ptr, datastructure::ApplicationInputs &inputs);
  bool setLinearVelocityFlagsInApplicationInputs(const BYTE *data_ptr, datastructure::ApplicationInputs &inputs);
  bool setSleepModeInputInApplicationInputs(const BYTE *data_ptr, datastructure::ApplicationInputs &inputs);
  bool setEvalutaionPathsOutputsInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setEvaluationPathsOutputsEvalOutInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setEvaluationPathsOutputsIsSafeInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setEvaluationPathsOutputsValidFlagsInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setMonitoringCaseOutputsInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setMonitoringCaseNumbersInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setMonitoringCaseFlagsInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setSleepModeOutputInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setErrorFlagsInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setLinearVelocityOutoutsInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setLinearVelocity0InApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setLinearVelocity1InApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setLinearVelocityFlagsInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setResultingVelocityOutputsInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setResultingVelocityInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setResultingVelocityFlagsInApplicationOutputs(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
  bool setOutputFlagsinApplicationOutput(const BYTE *data_ptr, datastructure::ApplicationOutputs &outputs);
};

}
}


