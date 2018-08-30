#include <sick_microscan3_ros_driver/datastructure/Data.h>

namespace sick {
namespace datastructure {

Data::Data()
{

}

boost::shared_ptr<DataHeader> Data::getDataHeaderPtr() const
{
  return m_data_header_ptr;

}


void Data::setDataHeaderPtr(const boost::shared_ptr<DataHeader> &data_header_ptr)
{
  m_data_header_ptr = data_header_ptr;
}

boost::shared_ptr<GeneralSystemState> Data::getGeneralSystemStatePtr() const
{
  return m_general_system_state_ptr;
}

void Data::setGeneralSystemStatePtr(const boost::shared_ptr<GeneralSystemState> &general_system_state_ptr)
{
  m_general_system_state_ptr = general_system_state_ptr;
}

boost::shared_ptr<DerivedValues> Data::getDerivedValuesPtr() const
{
  return m_derived_values_ptr;
}

void Data::setDerivedValuesPtr(const boost::shared_ptr<DerivedValues> &derived_values_ptr)
{
  m_derived_values_ptr = derived_values_ptr;
}

boost::shared_ptr<MeasurementData> Data::getMeasurementDataPtr() const
{
  return m_measurement_data_ptr;
}

void Data::setMeasurementDataPtr(const boost::shared_ptr<MeasurementData> &measurement_data_ptr)
{
  m_measurement_data_ptr = measurement_data_ptr;
}

boost::shared_ptr<IntrusionData> Data::getIntrusionDataPtr() const
{
  return m_intrusion_data_ptr;
}

void Data::setIntrusionDataPtr(const boost::shared_ptr<IntrusionData> &intrusion_data_ptr)
{
  m_intrusion_data_ptr = intrusion_data_ptr;
}

boost::shared_ptr<ApplicationInputs> Data::getApplicationInputsPtr() const
{
  return m_application_inputs_ptr;
}

void Data::setApplicationInputsPtr(const boost::shared_ptr<ApplicationInputs> &application_inputs_ptr)
{
  m_application_inputs_ptr = application_inputs_ptr;
}

boost::shared_ptr<ApplicationOutputs> Data::getApplicationOutputsPtr() const
{
  return m_application_outputs_ptr;
}

void Data::setApplicationOutputsPtr(const boost::shared_ptr<ApplicationOutputs> &application_outputs_ptr)
{
  m_application_outputs_ptr = application_outputs_ptr;
}

}
}

