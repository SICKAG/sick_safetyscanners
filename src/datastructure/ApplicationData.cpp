#include <sick_microscan3_ros_driver/datastructure/ApplicationData.h>

namespace sick {
namespace datastructure {

ApplicationData::ApplicationData()
{

}

ApplicationInputs ApplicationData::getInputs() const
{
  return m_inputs;
}

void ApplicationData::setInputs(const ApplicationInputs &inputs)
{
  m_inputs = inputs;
}

ApplicationOutputs ApplicationData::getOutputs() const
{
  return m_outputs;
}

void ApplicationData::setOutputs(const ApplicationOutputs &outputs)
{
  m_outputs = outputs;
}



}
}

