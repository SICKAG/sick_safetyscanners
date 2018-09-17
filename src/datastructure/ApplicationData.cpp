#include <sick_microscan3_ros_driver/datastructure/ApplicationData.h>

namespace sick {
namespace datastructure {

ApplicationData::ApplicationData()
  : m_is_empty(false)
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

bool ApplicationData::isEmpty() const
{
  return m_is_empty;
}

void ApplicationData::setIsEmpty(bool is_empty)
{
  m_is_empty = is_empty;
}



}
}

